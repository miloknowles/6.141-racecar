import cv2
import imutils
import numpy as np
import pdb
# from matplotlib import pyplot as plt
# from matplotlib import patches as patches


def cd_sift_ransac(img, debug=False):
    """
    Implement the cone detection using SIFT + RANSAC algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """

    # Note: (milo) I tried using crossCheck=True and bf.match (rather than bf.knnMatch),
    # but the feater matches that this returned seemed even worse.

    # Minimum number of matching features.
    MIN_MATCH = 8

    # Open template image.
    img_template  = cv2.imread('./test_images/cone_template.png', 0)

    # Create SIFT detector.
    sift = cv2.xfeatures2d.SIFT_create()

    # Compute SIFT on template and test image.
    kp1, des1 = sift.detectAndCompute(img_template, None)
    kp2, des2 = sift.detectAndCompute(img, None)

    # Create the brute force matcher.
    bf = cv2.BFMatcher(normType=cv2.NORM_L2, crossCheck=False)

    # Matches is a list of k-length lists of DMatch objects.
    # For each feature in image 1, finds the k best matches in image two.
    matches = bf.knnMatch(des1, des2, k=2)

    img3 = np.copy(img)

    if debug:
        img3 = cv2.drawMatchesKnn(img_template, kp1, img, kp2, matches, img3, flags=2)
        cv2.imshow('Best Matches', img3)
        cv2.waitKey(0)

    # Find and store good matches using D. Lowe's ratio test.
    # It requires that the cost of the best match is some ratio
    # below the second best match.
    good = []
    for m, n in matches:
        if m.distance < 0.80*n.distance: # Change this back to 0.8
            good.append(m)

    if debug:
        img3 = np.copy(img)
        img3 = cv2.drawMatchesKnn(img_template, kp1, img, kp2, [[m] for m in good], img3, flags=2)
        cv2.imshow('Best Matches', img3)
        cv2.waitKey(0)
        print('Best:', len(good))

    # If enough good matches, find bounding box.
    # Currently, with the test images and threshold, there are not enough
    # matches to proceed...
    if len(good) > MIN_MATCH:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        # Create mask
        max_reprojection_threshold = 10.0
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, max_reprojection_threshold)
        matchesMask = mask.ravel().tolist()

        h, w = img_template.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        ########## YOUR CODE STARTS HERE ##########

        # Perform perspectiveTransform with points and mask
        dst = cv2.perspectiveTransform(pts,M)
        img2 = cv2.polylines(img2,[np.int32(dst)], True, 255, 3, cv2.LINE_AA)

        # Find bounding box coordinates
        x_min = 0
        y_min = 0
        x_max = 0
        y_max = 0
        ########### YOUR CODE ENDS HERE ###########

        # Return bounding box
        return ((x_min, y_min), (x_max, y_max))
    else:

        # print "not enough matches; matches: ", len(good)

        # Return bounding box of area 0 if no match found
        return ((0,0), (0,0))

def cd_template_matching(img):
    """
    Implement the cone detection using template matching algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # Option for edge_detection
    EDGE_DETECTION = True

    # Import template image
    template = cv2.imread('./test_images/cone_template.png')

    # Perform Canny Edge detection on template
    if EDGE_DETECTION:
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template = cv2.Canny(template, 50, 200)

    # Get dimensions of template
    (template_height, template_width) = template.shape[:2]

    # Keep track of best-fit match
    best_match = None

    # Loop over different scales of image
    for scale in np.linspace(1.0, 0.02, 50):

        # Resize the image
        resized_img = imutils.resize(img, width = int(img.shape[1] * scale))
        ratio = img.shape[1] / float(resized_img.shape[1])

        # Check to see if test image is now smaller than template image
        if resized_img.shape[0] < template_height or resized_img.shape[1] < template_width:
            break

        # Perform Canny Edge detection on test image
        if EDGE_DETECTION:
            resized_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
            resized_img = cv2.Canny(resized_img, 50, 200)

        result = cv2.matchTemplate(resized_img, cv2.CV_TM_SQDIFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc

        bottom_right = (top_left[0] + w, top_left[1] + h)

        cv2.rectangle(img,top_left, bottom_right, 255, 2)


        ########## YOUR CODE STARTS HERE ##########

        # Apply OpenCV template match and find coefficient/location

        # Update best fitting match with max_cal, max_loc, ratio

        ########### YOUR CODE ENDS HERE ###########

    # Use best fitting match and locate object using ratio
    if best_match is not None:
        (max_val, max_loc, ratio) = best_match
        bb_start = (int(max_loc[0] * ratio), int(max_loc[1] * ratio))
        bb_end = (int(bb_start[0] + template_width*ratio), int(bb_start[1] + template_height*ratio))

        # Create bounding box
        bounding_box = (bb_start, bb_end)
    else:
        # No match found
        bounding_box = ((0,0), (0,0))

    return bounding_box


def filter_contours(frame, height, width, ratio=0.01):
    imgray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    image, contours, hierarchy = cv2.findContours(imgray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    minArea = ratio*height*width
    realContours = []
    for cnt in contours:
        cntArea = cv2.contourArea(cnt)
        if cntArea>=minArea:
            realContours.append(cnt)

    return realContours


def get_contour_centroid(cnt):
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        Cx = int(M['m10']/M['m00'])
        Cy = int(M['m01']/M['m00'])
        return Cx, Cy
    else:
        return None


def cd_color_segmentation_contours(img, debug=False):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    height, width, channels = img.shape

    # Filter the "red" stuff out of the frame using ratios against other colors.
    filteredFrameRed = np.zeros((height,width,3),np.uint8)
    b,g,r = cv2.split(img)

    redGreenRatio = 1.4
    redBlueRatio = 1.4
    filteredFrameRed[((r>(redGreenRatio*g)) & (r>(redBlueRatio*b)))] = [0,0,255]

    if debug:
        # Show original image.
        cv2.imshow('Red Threshold', filteredFrameRed)
        cv2.waitKey(0)

    # Extract contours from the color filtered image.
    # Sort them by size (first is largest).
    ratio = 0.01
    contours = filter_contours(filteredFrameRed, height, width, ratio=ratio)

    tries = 1
    maxRetries = 10
    while (tries < maxRetries and len(contours) == 0):
        ratio /= 2
        contours = filter_contours(filteredFrameRed, height, width, ratio=ratio)
        tries += 1

    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if len(contours) > 0:
        # Get the x, y coordinates of the contour center of mass.
        Cx, Cy = get_contour_centroid(contours[0])

        # Find bounding box coordinates.
        minx, miny, w, h = cv2.boundingRect(contours[0])

        # Find the min area rectangle (can be rotated).
        rect = cv2.minAreaRect(contours[0])
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Convert the topleft corner given by boundingRect into bottom left.
        x1 = minx
        y1 = miny
        x2 = minx + w
        y2 = miny + h

        if debug:
            # Draw the largest contour and its centroid on the image.
            img = cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
            img = cv2.circle(img, (Cx, Cy), 10, (255, 0, 0), thickness=-1)

            # Draw the min area rectangle.
            cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

            # Draw the vertical-horizontal bounding box.
            cv2.rectangle(img, (minx, miny), (minx+w, miny+h), (0, 255, 0), 2)
            cv2.imshow('Largest Red Contour', img)
            cv2.waitKey(0)

        # Return bounding box
        return ((x1, y1), (x2, y2))
    else:
        return ((0,0),(0,0))


def cd_color_segmentation_hsv(img, debug=False):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    HSV_MIN = np.array([6,225,120])
    HSV_MAX = np.array([35,255,255])
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Boolean map of orange pixels
    orig_orange_bool = cv2.inRange(hsv_img, HSV_MIN, HSV_MAX)
    kernel_small = np.ones((2,2),np.uint8)
    kernel_large = np.ones((12,12),np.uint8)

    # Fill in tiny holes then remove small regions of orange
    orange_bool = cv2.dilate(orig_orange_bool, kernel_small)
    orange_bool = cv2.erode(orange_bool, kernel_large)
    orange_bool = cv2.dilate(orange_bool, kernel_large)

    # Find the intersection with originally seen orange pixels
    dilated_orange_bool = cv2.dilate(orange_bool, kernel_large)
    cone_bool = cv2.bitwise_and(dilated_orange_bool, orig_orange_bool)
    cone_points = np.array(np.where(cone_bool)).T
    # dilated_points = np.array(np.where(dilated_orange_bool)).T
    # orig_points = np.array(np.where(orig_orange_bool)).T
    # points = np.array(np.where(orange_bool)).T

    y1, x1, h, w = cv2.boundingRect(cone_points)
    x2 = x1 + w
    y2 = y1 + h

    # if debug:
    #    fig, ax = plt.subplots(1)
    #    ax.imshow(img[:, :, [2,1,0]])
    #    rect = patches.Rectangle((x1,y1),w,h,linewidth=1,edgecolor='r',facecolor='none')
    #    ax.add_patch(rect)
        # ax.plot(dilated_points.T[1],dilated_points.T[0],'bo')
        # ax.plot(points.T[1],points.T[0],'ro')
        # ax.plot(cone_points.T[1],cone_points.T[0],'go')
        # ax.plot(cone_points.T[1],cone_points.T[0],'go',markerfacecolor=(1, 1, 0, 0.01))
        # plt.show()
        #
        # plt.imshow(hsv_img)
        # plt.imshow(img[:, :, [2,1,0]])
    #    plt.show()

    # Return bounding box
    return ((x1, y1), (x2, y2))

if __name__ == '__main__':
    # img = cv2.imread('./test_images/test9.jpg')
    # arr = np.array(img)
    # bbox = cd_color_segmentation_contours(arr, debug=True)
    # print(bbox)
    pass
