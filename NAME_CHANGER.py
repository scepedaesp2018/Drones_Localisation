import shutil
import os

files = os.listdir("./img")
for key in range(0, len(files)):
    print(files[key])
    os.rename('./img/'+str(files[key]), './img/drons'+str(key)+'.jpg')
