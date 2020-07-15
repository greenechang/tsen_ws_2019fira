import cv2
import numpy as np

MIN_AREA = 5000
MIN_AREA_CH = 1000
MAX_AREA = 100000

THRESHOLD_VALUE = 75

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
saliency = cv2.saliency.StaticSaliencyFineGrained_create()

def showImgs(imgs):
    for i, im in enumerate(imgs):
        cv2.imshow("Img_{}".format(str(i)), im)

    cv2.waitKey(1)

    #cv2.destroyAllWindows()
        
def findSquareContours(contours):
    results_i = []

    for i, cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA: # Too small, discard it
            continue

        perimeter = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.01 * perimeter, True)

        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            ratio = w / float(h)
            # Squares have aspect ratio close to 1
            if ratio >= 0.3 and ratio <= 1.7:
                results_i.append(i)

    return results_i

def getAllContoursSameHierarchy(hier, first_i):
    cnts_i = [first_i]

    i = hier[0, first_i, 0]
    while i != -1:
        cnts_i.append(i)
        i = hier[0, i, 0]
        if i == -1:
            break

    return cnts_i

def detectMarkers(img, debug=False):
    detected_markers = []

    # Convert to grayscale
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    

    #gray = cv2.subtract(hsv[:, :, 2], hsv[:, :, 1])
    gray = hsv[:, :, 2]
    # Apply some blurring to reduce noise
    #gray = cv2.blur(gray, (5, 5))
    # Regular hist equalization
    gray = cv2.equalizeHist(gray)

    #gray = (gray * 1.5).astype(np.uint8)

    # CLAHE histogram equalization
    #gray = clahe.apply(gray)

    ret, thresh = cv2.threshold(gray, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)
    #thresh_copy = thresh.copy() 
    #morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))


    #(success, saliencyMap) = saliency.computeSaliency(img)
    ##print("Success: {}".format(success))
    #saliencyMap = (saliencyMap * 255).astype("uint8")
    #ret, thresh = cv2.threshold(saliencyMap, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((5, 5)))

    # OpenCV 3.3.1-dev
    _, contours, hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # OpenCV 4.0
    #contours, hier = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None

    #cv2.drawContours(img, contours, -1, (0, 0, 255), 3)

    # These are the black boxes surrounding the markers
    father_indices = findSquareContours(contours)

    # Retrieve all same level contours

    for father_i in father_indices:
        # Retrieve father contour
        father_cnt = contours[father_i]
        rect = cv2.minAreaRect(father_cnt)
        angle_father = rect[2]

        if angle_father >= -90.0 and angle_father <= -45.0:
            angle_father = angle_father + 90
        angle_father *= -1

        #print(angle_father)

        first_child_i = hier[0, father_i, 2]
        children_cnts_i = getAllContoursSameHierarchy(hier, first_child_i)
        #print("Number of inside contours: {}".format(len(children_cnts_i)))
        children_cnts = [contours[i] for i in children_cnts_i]
        for ch_cnt in children_cnts:
            #ch_cnt = contours[ch_cnt_i]

            if cv2.contourArea(ch_cnt) < MIN_AREA_CH:
                continue

            # Need at least five vertices to fit an ellipse
            if len(ch_cnt) < 5:
                continue

            # Fit ellipse and find its angle
            ellipse = cv2.fitEllipse(ch_cnt)
            angle = ellipse[2]
            #print("Angle: {}".format(angle))

            #res_angle = angle - angle_father
            #print("Res angle: {}".format(res_angle))
            #print("Res angle: {}".format(res_angle % 90))

            if angle > 50 and angle < 90:
                detected_markers.append(("right", angle_father))
            elif angle > 100 and angle < 140:
                detected_markers.append(("left", angle_father))
            else:
                detected_markers.append(("straight", angle_father))

        if debug:
            cv2.drawContours(img, children_cnts, -1, (0, 255, 0), 3)

    if debug:
        showImgs([img, gray, thresh, hsv])
    
    return detected_markers

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        detected = detectMarkers(frame, debug=True)

        if detected != None and len(detected) > 0:
            print("[{}] -- {}".format(detected[0][0], detected[0][1]))
        #cv2.imshow("Video", frame)
        #cv2.waitKey(1)
