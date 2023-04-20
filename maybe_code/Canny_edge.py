import cv2
import numpy as np
path="home/ankit/Vision and learning based control/video/test.png"
image=cv2.imread(path)
image=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

gradients_sobelx=cv2.Sobel(image,-1,1,0)
gradients_sobely=cv2.Sobel(image,-1,0,1)
gradients_sobelxy=cv2.addWeighted(gradients_sobelx,0.5,gradients_sobely,0.5,0)
gradients_laplacian=cv2.Laplacian(image,-1)

canny_output=cv2.Canny(image,80,150)

cv2.imshow("Sobel_x",gradients_sobelx)
cv2.imshow("Sobel_y",gradients_sobely)
cv2.imshow("Sobel_x+y",gradients_sobelxy)
cv2.imshow("laplacian",gradients_laplacian)
cv2.imshow("canny",canny_output)
cv2.waitKey()



