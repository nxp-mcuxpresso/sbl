import os
import io
import sys
import shutil

# Setup sys path so 'imgtool' is in it.
FileDir = os.path.dirname(__file__)
FileDirABS = os.path.abspath(FileDir)
sys.path.insert(0, FileDirABS)

import imgtool.keys as keys

def main():
    if not os.path.exists('{}/sign-rsa2048-priv.pem'.format(FileDirABS)):
        os.system('python {0}/imgtool.py keygen -k {0}/sign-rsa2048-priv.pem -t rsa-2048-sign'.format(FileDirABS))
    os.system('python {0}/imgtool.py getpub -k {0}/sign-rsa2048-priv.pem -o sign-rsa2048-pub.pem -t sign'.format(FileDirABS))#{0}/
    if os.path.exists('{}/../mcuboot/portable/sign-rsa2048-pub.c'.format(FileDirABS)):
        os.remove('{}/../mcuboot/portable/sign-rsa2048-pub.c'.format(FileDirABS))
    shutil.move('{}/sign-rsa2048-pub.c'.format(FileDirABS), '{}/../mcuboot/portable'.format(FileDirABS))

    if not os.path.exists('{}/enc-rsa2048-priv.pem'.format(FileDirABS)):
        os.system('python {0}/imgtool.py keygen -k enc-rsa2048-priv.pem -t rsa-2048-enc'.format(FileDirABS))#{0}/
    else:
        k = keys.load('enc-rsa2048-priv.pem')#{}/ #.format(FileDirABS)
        ccode = io.StringIO()
        k.emit_c(file=ccode, type='priv')
        file = open('enc-rsa2048-priv.c', 'w')#{}/ #.format(FileDirABS)
        file.write(ccode.getvalue())
        file.close()
        print(ccode.getvalue())
    os.system('python {0}/imgtool.py getpub -k {0}/enc-rsa2048-priv.pem -o enc-rsa2048-pub.pem -t enc'.format(FileDirABS))#{0}/
    if os.path.exists('{}/../mcuboot/portable/enc-rsa2048-priv.c'.format(FileDirABS)):
        os.remove('{}/../mcuboot/portable/enc-rsa2048-priv.c'.format(FileDirABS))
    shutil.move('{}/enc-rsa2048-priv.c'.format(FileDirABS), '{}/../mcuboot/portable'.format(FileDirABS))

if __name__ == '__main__':
    main()