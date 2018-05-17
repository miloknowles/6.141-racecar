import cv2
import numpy as np
import csv
import ast
import sys
from cone_detection import cd_sift_ransac, cd_template_matching, cd_color_segmentation

def iou_score(bbox1, bbox2):
    """
    Return the IoU score for the two bounding boxes
    Input:
        bbox1: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
        bbox2: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    Return:
        score: float; the IoU score
    """

    # Determine intersection rectangle
    x_int_1 = max(bbox1[0][0], bbox2[0][0])
    y_int_1 = max(bbox1[0][1], bbox2[0][1])
    x_int_2 = min(bbox1[1][0], bbox2[1][0])
    y_int_2 = min(bbox1[1][1], bbox2[1][1])

    # Compute area of intersection
    area_int = (x_int_2 - x_int_1 + 1) * (y_int_2 - y_int_1 + 1)

    # Compute area of both bounding boxes
    area_bbox1 = (bbox1[1][0] - bbox1[0][0] + 1) * (bbox1[1][1] - bbox1[0][1] + 1)
    area_bbox2 = (bbox2[1][0] - bbox2[0][0] + 1) * (bbox2[1][1] - bbox2[0][1] + 1)

    # Compute area of union
    area_union = float(area_bbox1 + area_bbox2 - area_int)

    # Compute and return IoU score
    score = area_int / area_union

    # Reject negative scores
    if score < 0:
        score = 0

    return score

def test_algorithm(detection_func, test_file_path = "./test_images.csv"):
    """
    Test a cone detection function and return the average score based on all the test images
    Input:
        detection_func: func; the cone detection function that takes the np.3darray
                as input and return (bottom, left, top, right) as output
        test_file_path: string; the path to the test file
    Return:
        scores: dict; the score for each test image
    """
    # Keep track of scores
    scores = {}

    # Open test images csv
    with open(test_file_path) as csvDataFile:
        csvReader = csv.reader(csvDataFile)
        # Iterate through all test images
        for row in csvReader:
            # Find image path and ground truth bbox
            img_path = row[0]
            bbox_true = ast.literal_eval(row[1])
            print('True:', bbox_true)
            img = cv2.imread(img_path)

            # Detection bbox
            bbox_est = detection_func(img)
            print('Estimated:', bbox_est)
            score = iou_score(bbox_est, bbox_true)
            
            # Add score to dict
            scores[img_path] = score

    # Return scores
    return scores

def test_all_algorithms(test_file_path = "./test_images.csv", output_file_path = './test_scores.csv'):
    """
    Test all algorithms and write score results to csv file
    Input:
        test_file_path: string; the path to the test file
    Output:
        test_scores: csvfile; the scores for each image for each algorithm
    """

    try:
        scores_sift = test_algorithm(cd_sift_ransac, test_file_path)
        sift_data = [['SIFT', img, score] for img, score in scores_sift.iteritems()]
    except:
        sift_data = []

    try:
        scores_template = test_algorithm(cd_template_matching, test_file_path)
        template_data = [['template_matching', img, score] for img, score in scores_template.iteritems()]
    except:
        template_data = []
    
    try:
        print('Trying color segmentation test.')
        scores_color_seg = test_algorithm(cd_color_segmentation, test_file_path)
        color_seg_data = [['color_segmentation', img, score] for img, score in scores_color_seg.iteritems()]
    except:
        print('Exception')
        color_seg_data = []

    score_data = sift_data + template_data + color_seg_data

    output_file = open(output_file_path, 'w')  
    with output_file:  
        writer = csv.writer(output_file)
        writer.writerows(score_data)

    print("Scores outputted to %s" % (output_file_path))

if __name__ == '__main__':
    if len(sys.argv) == 1:
        test_all_algorithms()
    else:
        for arg in sys.argv[1:]:
            scores = None

            if arg == "color":
                scores = test_algorithm(cd_color_segmentation)
            elif arg == "template":
                scores = test_algorithm(cd_template_matching)
            elif arg == "sift":
                scores = test_algorithm(cd_sift_ransac)
            else:
                print("Argument '%s' not recognized" % (arg))

            if scores:
                for (img, val) in scores.iteritems():
                    print((img, val))
