# Classe decode
    # função: imagem(cortada), bounding box(x, y, h, w), tipo -> (coordenadas, valor, lido, tipo)
        # filtro para escala de cinza
        # leitor
import cv2
from pyzbar import pyzbar
import numpy as np

def read_qrcode(image, bounding_box):
    # convert to gray scale
    image = cv2.resize(image, (512, 512))
    cv2.imshow( "resize", image)
    cv2.waitKey()
    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    cv2.imshow("gray", image)
    cv2.waitKey()

    image = cv2.GaussianBlur(image, (7,7), 0)
    cv2.imshow("blur", image)
    cv2.waitKey()

    (thresh, im_bw) = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY)
    im2 = cv2.morphologyEx(im_bw,cv2.MORPH_CLOSE,(3,3))

    cv2.imshow("bw", im2)
    cv2.waitKey()


    # read qrcode
    qrcodes = pyzbar.decode(im2)
    if len(qrcodes) > 0:
        print(qrcodes[0])
        qrcode_info = qrcodes[0].data.decode('utf-8')
        return (qrcode_info, bounding_box, True)

    return (None, bounding_box, False)

if __name__ == '__main__':
    image = cv2.imread('qrcode1.jpg')
    cv2.imshow("image", image)
    cv2.waitKey()
    
    bb = (1.3, 0, 5, 7)
    print(read_qrcode(image, bb))