import ftplib

def Upload(s):
    "UPLOADS A FILE TO THE SERVER"
    session = ftplib.FTP('192.168.168.1','drone','NEVERAGAIN')
    file = open(s,'rb')                  # file to send
    session.storbinary('STOR ' + s, file)     # send the file
    file.close() 
    session.quit()
    return
