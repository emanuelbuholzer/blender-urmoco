import sys

from scipy.linalg import norm
from scipy import sum, average
import cv2

def main():
    file1, file2 = sys.argv[1:1+2]
    # read images as 2D arrays (convert to grayscale for simplicity)
    img1 = to_grayscale(cv2.imread(file1).astype(float))
    img2 = to_grayscale(cv2.imread(file2).astype(float))
    n_m = compare_images(img1, img2)
    print("Manhattan norm:", n_m, "/ per pixel:", n_m/img1.size)

    cv2.imwrite('diff.jpg', cv2.bitwise_not(cv2.absdiff(img1, img2)))
    print("Wrote difference image diff.jp")

def compare_images(img1, img2):
    img1 = normalize(img1)
    img2 = normalize(img2)
    diff = img1 - img2
    m_norm = sum(abs(diff))  # Manhattan norm
    return m_norm

def to_grayscale(arr):
    if len(arr.shape) == 3:
        return average(arr, -1)  # average over the last axis (color channels)
    else:
        return arr

def normalize(arr):
    rng = arr.max()-arr.min()
    amin = arr.min()
    return (arr-amin)*255/rng

if __name__ == "__main__":
    main()
