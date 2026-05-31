import cv2
import cv2.aruco as aruco
import os

output_dir = r"D:\Reto\ARUCO"
os.makedirs(output_dir, exist_ok=True)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

for i in range(4):  # genera 4 markers: 0,1,2,3
    marker = aruco.generateImageMarker(dictionary, id=i, sidePixels=300)
    filename = os.path.join(output_dir, f"marker_{i}.png")
    cv2.imwrite(filename, marker)

print("Markers generados correctamente")