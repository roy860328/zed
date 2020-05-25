import cv2
import numpy as np

img = cv2.imread("1_7_rgb_706_1589382101.png")
img = cv2.imread("1_7_depth_706_1589382101.jpg")
edges = cv2.Canny(img,90,110)
edges = cv2.Canny(img,50,60)
edges = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)
# row,col,ch= edges.shape
# mean = 0
# var = 0.05
# sigma = var**0.5
# gauss = np.random.normal(mean,sigma,(row,col,ch))
# gauss = gauss.reshape(row,col,ch)
# noisy = edges + gauss

print(edges.shape)
cv2.imshow("result", img)
# cv2.imshow("noisy", noisy)
cv2.waitKey(0)
cv2.destroyAllWindows()
