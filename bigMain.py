#Copyright (c) 2019 UofRobotics

#Python RPLidar Visualisation Server over MJPEG
#Compatible with VLC as HTTP stream
#Server Runs on Ubuntu, Requires RPLidar

from adafruit_rplidar import RPLidar, RPLidarException
from PIL import Image, ImageDraw
import io
import time
import math
import socket
import multiprocessing
from multiprocessing import shared_memory

ip='0.0.0.0' #Don't Change This
port=8081 #Hosting Port, Don't Change This
buff=1500 #Also Don't Change This
port_name='/dev/ttyUSB0' #Also Don't Change This

#Initalisation of TCP Socket Server
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM) #TCP/IP Socket
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) #Unbind when Done
s.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1) #Zero-Latency TCP
s.bind((ip,port)) #Start Server
s.listen(1) #Listen for Connections

#Initalisation of Log File
f=open('lidarVidDebug.log','a') #Don't Change This

#Mandatory HTTP Headers (Required for Functionality)
iostr="\r\n--R2lpaXUgU3dmYW5iY2o0\r\nContent-Type: image/jpeg\r\nContent-Length: ".encode("utf-8")
estr="\r\n\r\n".encode("utf-8")

#Function Call Speedups
tc=time.perf_counter
tt=time.time
mf=math.floor
mc=math.cos
ms=math.sin
ts=time.sleep
bio=io.BytesIO
fw=f.write
ff=f.flush
st=s.settimeout
ste=socket.timeout
rdwr=socket.SHUT_RDWR
se=socket.error
pi=math.pi

#Initalisation of Address and Lidar Array
print("READY!")
addr=["NC","NC"]

#Definitions

#Core Reciever
def get_data(smq,sma,smd,sml,sms,port_name):
    lidar=RPLidar(None,port_name)
    lis=lidar.iter_scans
    for scan in lis():
        if sms[0]: break
        sml[0]=int(len(scan))
        for x in range(0,sml[0]):
            n=scan[x]
            smq[x]=n[0]
            sma[x]=n[1]
            smd[x]=n[2]
    lidar.stop()
    lidar.set_pwm(0)
    lidar.disconnect()
    print('SCAN STOPPED!')

#Renderer
def process_data(smq,sma,smd,sml,dp):
    #Quality, Angle, Distance
    #Quality is 0-15
    #Angle is 0.0-360.0
    #Distance is 0.0-12000.0
    mdist=max(smd) #Auto-Scaling
    if mdist>0:
        if sml[0]>360:
            q=(360/sml[0])*(360/(2*pi))
        else:
            q=360/(2*pi)
        for x in range(0,sml[0]):
            dist=(smd[x]/mdist)*255 #Auto-Scale
            w=mf((mc(sma[x]/q)*dist)+256)
            h=mf((ms(sma[x]/q)*dist)+256)
            g=mf((smq[x])*17)
            r=255-g
            dp((w,h),(r,g,255))
        
#Now the Actual Code

#Run Before Connect
smq=shared_memory.ShareableList([0.0]*400)
sma=shared_memory.ShareableList([0.0]*400)
smd=shared_memory.ShareableList([0.0]*400)
sml=shared_memory.ShareableList([0])
sms=shared_memory.ShareableList([False])
p1=multiprocessing.Process(target=get_data,args=(smq,sma,smd,sml,sms,port_name),daemon=True)
p1.start()

#Continuity Loop
while True:
    
    #Do Not Change Anything Below, All of this is Security
    print("Disconnected")
    print(tt())
    print(addr)
    fw("Disconnected\n")
    fw(str(tt())+"\n")
    fw(str(addr[0])+", ")
    fw(str(addr[1])+"\n")
    ff()
    
    #Ctrl-C Handler
    st(None)
    try:
        conn,addr=s.accept()
    except KeyboardInterrupt:
        break
    
    print("Connected")
    print(tt())
    print(addr)
    fw("Connected\n")
    fw(str(tt())+"\n")
    fw(str(addr[0])+", ")
    fw(str(addr[1])+"\n")
    ff()
    ct=conn.settimeout
    ct(1.0)
    #Do Not Change Anything Above
    
    
    #Header Communication with Client
    cs=conn.sendall #Connection Speedup
    #Client Timeout Handler
    try:
        data=conn.recv(buff)
    except ste:
        print("BOT!")
        fw("BOT!\n")
        conn.shutdown(rdwr)
        conn.close()
        continue
    except se:
        conn.shutdown(rdwr)
        conn.close()
        continue
    
    #Also Mandatory HTTP Headers
    ostr="HTTP/1.1 200 OK\r\nConnection: close\r\nServer: PyVidStreamServer MJPEG SERVER\r\nCache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\r\nPragma: no-cache\r\nExpires: -1\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace;boundary=R2lpaXUgU3dmYW5iY2o0\r\n\r\n"

    o=ostr.encode("utf-8")
    #Server Connection Failure Handler
    try:
        cs(o) #Sending Header
    except ste:
        print("BOT!")
        fw("BOT!\n")
        conn.shutdown(rdwr)
        conn.close()
        continue
    except se:
        conn.shutdown(rdwr)
        conn.close()
        continue
    
    #Capture Loop
    while True:
        
        a=tc()#Start Time for Frame Limiting
                
        #Image Initialization goes here
        img=Image.new('RGB',(512,512)) #512,512 is good size for split screens
        draw=ImageDraw.Draw(img)
        dp=draw.point
        dp((255,255),(0,0,255)) #RPLidar
                        
        #Data Processing with Concurrent Updating
        process_data(smq,sma,smd,sml,dp)
        
        #Converting Raw Image into Compressed JPEG Bytes
        with bio() as output:
            img.save(output,format="JPEG",quality=25)
            contents=output.getvalue()
            
        #Concatenating Contents and Headers
        o=iostr
        o+=str(len(contents)).encode("utf-8")
        o+=estr
        o+=contents
        
        #Sending Contents to Client
        try:
            cs(o)
        except:
            break
            
        #frame rate limiter
        b=tc() #End Time for Frame Limiting
        c=b-a
        t=1/30 #seconds per frame
        if t-c>0.0:
            ts(t-c) #delay remaining seconds
        elif c>t:
            pass
            #print(c)
            
#End of Program (when Ctrl-C)
f.close()
s.close()
#kill lidar
sms[0]=True
ts(1.0)
smq.close()
smq.unlink()
sma.close()
sma.unlink()
smd.close()
smd.unlink()
sml.close()
sml.unlink()
sms.close()
sms.unlink()
