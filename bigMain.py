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
from multiprocessing import shared_memory, Lock

ip='0.0.0.0' #Don't Change This
port=8081 #Hosting Port, Don't Change This
buff=1500 #Also Don't Change This
port_name='/dev/ttyUSB0' #Also Don't Change This
debug=False

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
def get_data(smqn,sman,smdn,smln,smsn,l,port_name):
    
    #Apperantly this is the Correct way to do Shared Memory
    smq=shared_memory.ShareableList(name=smqn)
    sma=shared_memory.ShareableList(name=sman)
    smd=shared_memory.ShareableList(name=smdn)
    sml=shared_memory.ShareableList(name=smln)
    sms=shared_memory.ShareableList(name=smsn)
    
    lidar=RPLidar(None,port_name)
    lis=lidar.iter_scans
    
    #Speeding up locks
    la=lock.acquire
    lr=lock.release
    while True: #Code Retry
        try:
            for scan in lis():
                la() #Locking
                if sms[0]: break #Graceful Shutdown Reciever
                sml[0]=int(len(scan))
                for x in range(0,sml[0]):
                    n=scan[x]
                    smq[x]=n[0]
                    sma[x]=n[1]
                    smd[x]=n[2]
                lr() #Unlocking
        except RPLidarException as e:
            print('RPLidarException')
            print(e)
            break
        except ValueError as e:
            print('Failure Due to Access Bug')
            print(e)
            break
        except KeyboardInterrupt as e:
            pass
       
    #End of Daemon     
    if l.locked(): lr() #allow code to run
    lidar.stop()
    lidar.set_pwm(0)
    lidar.disconnect()
    print('SCAN STOPPED!')

#Renderer
def process_data(smq,sma,smd,sml,calls,dp):
    #Quality, Angle, Distance
    #Quality is 0-15
    #Angle is 0.0-360.0
    #Distance is 0.0-12000.0
    if calls<2:
        try:
            mdist=max(smd) #Auto-Scaling
            if mdist>0:
                if sml[0]>360:
                    q=(360/sml[0])*(360/(2*pi)) #Auto-Adapt
                else:
                    q=360/(2*pi) #Auto-Adapt
                for x in range(0,sml[0]):
                    dist=(smd[x]/mdist)*255 #Auto-Scale
                    w=mf((mc(sma[x]/q)*dist)+256)
                    h=mf((ms(sma[x]/q)*dist)+256)
                    v=mf((smq[x])*17)
                    dp((w,h),(v,v,v)) #drawing
        except ValueError as e: #For Some Error Where Shared Memory Lists Decide that they have a Length of Zero for only one line
            if debug:
                print('Caught Exception')
                print(e)
                print('Dist')
                print(smd)
                print('Angle')
                print(sma)
                print('Quality')
                print(smq)
                print('Len')
                print(sml)
                print('FIN')
                print(len(smd))
                print(len(sma))
                print(len(smq))
                print(len(sml))
                print('Exception Handled')
            #Retry Frame Once
            process_data(smq,sma,smd,sml,calls+1,dp)
    else:
        if debug: print('Dropped Frame')
        pass
        
#Now the Actual Code

#Run Before Connect
l=Lock()
smq=shared_memory.ShareableList([0.0]*400) #Quality
sma=shared_memory.ShareableList([0.0]*400) #Angle
smd=shared_memory.ShareableList([0.0]*400) #Distance
sml=shared_memory.ShareableList([0]) #Current Array Length
sms=shared_memory.ShareableList([False]) #Stop Signal
#Apperantly You have to Pass the Names, not the Objects of the Shared Memory
p1=multiprocessing.Process(target=get_data,args=(smq.shm.name,sma.shm.name,smd.shm.name,sml.shm.name,sms.shm.name,l,port_name),daemon=True)
p1.start() #Start Daemon

#Continuity Loop
while True:
    
    #Do Not Change Anything Below, All of this is Security
    print("Disconnected")
    print(tt())
    print(addr)
    fw("Disconnected\n") #File Append
    fw(str(tt())+"\n")
    fw(str(addr[0])+", ")
    fw(str(addr[1])+"\n")
    ff() #Write to File
    
    #Ctrl-C Handler
    st(None) #Set Timeout
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
        if len(data)==0:
            print("BOT!")
            fw("BOT!\n")
            conn.shutdown(rdwr)
            conn.close()
            continue
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
    except KeyboardInterrupt:
        conn.shutdown(rdwr)
        conn.close()
        print("Disconnected")
        print(tt())
        print(addr)
        fw("Disconnected\n") 
        fw(str(tt())+"\n")
        fw(str(addr[0])+", ")
        fw(str(addr[1])+"\n")
        ff()
        break
    
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
    except KeyboardInterrupt:
        conn.shutdown(rdwr)
        conn.close()
        print("Disconnected")
        print(tt())
        print(addr)
        fw("Disconnected\n") 
        fw(str(tt())+"\n")
        fw(str(addr[0])+", ")
        fw(str(addr[1])+"\n")
        ff()
        break
    
    #Image Generation Only Once
    img=Image.new('RGB',(512,512)) #512,512 is good size for split screens
    draw=ImageDraw.Draw(img)
    dp=draw.point
    dr=draw.rectangle
    ims=img.save
    
    #Quick Pre-Loop Speedup, speeding up locks
    la=lock.acquire
    lr=lock.release
    
    #Capture Loop
    while True:
        
        a=tc() #Start Time for Frame Limiting
                
        #Image Initialization goes here
        dr([0,0,512,512],(0,0,0),(0,0,0)) #Black Out Image
        dp((255,255),(0,0,255)) #RPLidar Dot
                        
        #Data Processing with Concurrent Updating
        try:
            la() #Locking #was only clear in Python 2 Docs, that only acquire does the blocking for lock
            process_data(smq,sma,smd,sml,1,dp)
            lr() #Unlocking
        except KeyboardInterrupt:
            conn.shutdown(rdwr)
            conn.close()
            break
        
        #Converting Raw Image into Compressed JPEG Bytes
        try:
            with bio() as output:
                ims(output,format="JPEG",quality=75) #Quality of 50 is about Minimum
                contents=output.getvalue()
        except KeyboardInterrupt:
            conn.shutdown(rdwr)
            conn.close()
            break
            
        #Concatenating Contents and Headers
        o=iostr
        o+=str(len(contents)).encode("utf-8")
        o+=estr
        o+=contents
        
        #Sending Contents to Client
        try:
            cs(o)
        except ste:
            print("Network Issues!")
            fw("Network Issues!\n")
            conn.shutdown(rdwr)            
            conn.close()
            break
        except se:
            conn.shutdown(rdwr)
            conn.close()
            break
        except KeyboardInterrupt:
            conn.shutdown(rdwr)
            conn.close()
            break
            
            
        #frame rate limiter
        try:
            b=tc() #End Time for Frame Limiting
            c=b-a
            t=1/30 #seconds per frame
            if t-c>0.0:
                ts(t-c) #delay remaining seconds
            elif c>t:
                pass
                if debug: print(c)
                
        except KeyboardInterrupt:
            conn.shutdown(rdwr)
            conn.close()
            break
    
    #Proper Memory Usage
    img.close()
            
#End of Program (when Ctrl-C)
f.close()
s.close()

#Gracefully Kill RPLidar
la() #Locking
sms[0]=True #Graceful Shutdown Signal
lr() #Unlocking
p1.join() #Wait for Process to Finish
p1.close() #Destroy the Daemon Process Object

#Proper Shared Memory Closing
smq.shm.close()
smq.shm.unlink()
sma.shm.close()
sma.shm.unlink()
smd.shm.close()
smd.shm.unlink()
sml.shm.close()
sml.shm.unlink()
sms.shm.close()
sms.shm.unlink()
