contours = [5, 8, 10]
largest_contour = max(contours)
i=0
while i <= len(contours):
    if contours[i] == largest_contour:
        contours[i] = 0
        break
    i+=1

print(contours)