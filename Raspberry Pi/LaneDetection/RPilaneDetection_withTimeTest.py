import numpy as np
import logging
import math
import datetime
import sys
import cv2
import time

_SHOW_IMAGE = False
_ENFORCE_RESOLUTION = True


class HandCodedLaneFollower(object):
    
    # CONSTRUCTOR
    def __init__(self, car=None):
        #logging.info('Creating a HandCodedLaneFollower...')
        self.car = car
        self.curr_steering_angle = 90

    # FINDS & PROCESSES LANE-LINES
    def follow_lane(self, frame):
        # Main entry point of the lane follower
        show_image("orig", frame)
        lane_lines_arr, frame = detect_lane(frame)
        final_frame = self.steer(frame, lane_lines_arr)
        return final_frame
    
    # COMPUTES STEERING DIRECTION
    def steer(self, frame, lane_lines_arr):
        #logging.debug('steering...')
        if len(lane_lines_arr) == 0:
            #logging.error('No lane lines detected, nothing to do.')
            return frame
            
        new_steering_angle = compute_steering_angle(frame, lane_lines_arr)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines_arr))
        
        # Send steering-signal to Arduino
        if self.car is not None:
            self.car.front_wheels.turn(self.curr_steering_angle)
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        show_image("heading", curr_heading_image)

        return curr_heading_image


############################
# Frame processing steps
############################

'''LANE-DETECTION MASTER'''
def detect_lane(frame):

    e1 = cv2.getTickCount()
    #logging.debug('detecting lane lines...')
    # EDGE DETECTION
    edges = detect_edges(frame)
    show_image('edges', edges)
    # REMOVE IRRELEVANCIES
    cropped_edges = region_of_interest(edges)
    show_image('edges cropped', cropped_edges)
    # LINE-DETECTION
    line_segments_arr = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments_arr)
    show_image("line segments", line_segment_image)
    # LANE-LINE CONSTRUCTION
    lane_lines_arr = average_slope_intercept(frame, line_segments_arr)
    lane_lines_image = display_lines(frame, lane_lines_arr)
    show_image("lane lines", lane_lines_image)

    e2 = cv2.getTickCount()
    time = (e2-e1)/cv2.getTickFrequency()
    print("detect_lane: ", time)

    return lane_lines_arr, lane_lines_image

def detect_edges(frame):

    e1 = cv2.getTickCount()
    # only check pixels that're the color of lane-lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    show_image("hsv", hsv)
    mask = cv2.inRange(hsv, np.array([30, 40, 0]) , np.array([150, 255, 255]))
    # detect the remaining edges in the image, including lane-lines
    edges = cv2.Canny(mask, 200, 400)
    show_image("blue mask", mask)
    e2 = cv2.getTickCount()
    time = (e2-e1)/cv2.getTickFrequency()
    print("detect_edges: ", time)
    return edges

# lane-lines are in bottom half of frame
def region_of_interest(canny):

    e1 = cv2.getTickCount()
    height, width = canny.shape
    mask = np.zeros_like(canny)
    # 4-corners of top-half
    polygon = np.array([[ 
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)
    # white-out top-half of frame
    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    show_image("mask", mask)

    e2 = cv2.getTickCount()
    time = (e2-e1)/cv2.getTickFrequency()
    print("region_of_interest: ", time)

    return masked_image

# find LINES in frame
def detect_line_segments(cropped_edges):
    '''
    No mask will cleanly identify the 2 lane-lines. 
    Instead, the mask detects a multiple lane-segments for each lane.
    '''

    e1 = cv2.getTickCount()
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments_arr = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)
    #if line_segments_arr is not None:
     #   for line_segment in line_segments_arr:
            #logging.debug('detected line_segment:')
            #logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    e2 = cv2.getTickCount()
    time = (e2-e1)/cv2.getTickFrequency()
    print("detect_line_segments: ", time)
    return line_segments_arr

# derive LANE-LINES from identified line-segments
def average_slope_intercept(frame, line_segments_arr):

    e1 = cv2.getTickCount()
    """
    This function combines line-segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines_arr = []
    if line_segments_arr is None:
        #logging.info('No line_segment segments detected')
        return lane_lines_arr

    height, width, _ = frame.shape
    left_fit_segments = []      # create L-lane from scattered segments (see image in README.md)
    right_fit_segments = []     # create R-lane from scattered segments
                                # (the line in the middle is artificially generated later on)
                            
    # constrain detection of L & R lanes
    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # L lane-line is on left 2/3 of screen
    right_region_boundary = width * boundary       # R lane line is right 2/3 (the area outside this bound)
    
    # use SLOPE to differentite L&R lane-segments
    for line_segment in line_segments_arr:
        for x1, y1, x2, y2 in line_segment:        
            if x1 == x2:
                #logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            ee1 = cv2.getTickCount()
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            ee2 = cv2.getTickCount()
            # print("polyfit : ", (ee2-ee1)/cv2.getTickFrequency())
            slope = fit[0]
            intercept = fit[1]
            # left lane
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit_segments.append((slope, intercept))
            # right lane
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit_segments.append((slope, intercept))
   
    # construct lanes
    left_fit_average = np.average(left_fit_segments, axis=0)
    if len(left_fit_segments) > 0:
        lane_lines_arr.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit_segments, axis=0)
    if len(right_fit_segments) > 0:
        lane_lines_arr.append(make_points(frame, right_fit_average))

    #logging.debug('lane lines: %s' % lane_lines_arr)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    e2 = cv2.getTickCount()
    time = (e2 - e1)/cv2.getTickFrequency()
    print("average_slope_intercept :", time)

    return lane_lines_arr


''' Steering Functionality '''
def compute_steering_angle(frame, lane_lines_arr):
    """ Find the steering angle based on lane line coordinate
    
        **************** We assume that camera is calibrated to point to dead center ****************
    
    """

    if len(lane_lines_arr) == 0:
        #logging.info('No lane lines detected')
        return -90
    
    e1 = cv2.getTickCount()
    height, width, _ = frame.shape

    if len(lane_lines_arr) == 1:
        #logging.debug('Only detected one lane line, just follow it. %s' % lane_lines_arr[0])
        x1, _, x2, _ = lane_lines_arr[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines_arr[0][0]
        _, _, right_x2, _ = lane_lines_arr[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    #logging.debug('new steering angle: %s' % steering_angle)
    e2 = cv2.getTickCount()
    time = (e2 - e1)/ cv2.getTickFrequency()
    print("compute_steering_angle :", time)
    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """

    e1 = cv2.getTickCount()

    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    #logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))

    e2 = cv2.getTickCount()
    time = (e2 - e1)/ cv2.getTickFrequency()
    print("stabilize_steering_angle :", time)

    return stabilized_steering_angle


############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)

def make_points(frame, line):
    
    e1 = cv2.getTickCount()

    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

    e2 = cv2.getTickCount()
    time = (e2 - e1)/ cv2.getTickFrequency()
    print("make_points:", time)
    return [[x1, y1, x2, y2]]

############################
# Test Functions
############################
def test_photo(file):
    land_follower = HandCodedLaneFollower()
    frame = cv2.imread(file)
    combo_image = land_follower.follow_lane(frame)
    show_image('final', combo_image, True)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_video(video_file):
    lane_follower = HandCodedLaneFollower()
    # cap = cv2.VideoCapture(video_file + '.avi')
    cap = cv2.VideoCapture(0)
    
    # lower RESOLUTION for faster processing [https://picamera.readthedocs.io/en/release-1.12/fov.html]
    if _ENFORCE_RESOLUTION:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640);
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480);

    # skip first second of video.
    for i in range(3):
        _, frame = cap.read()

    video_type = cv2.VideoWriter_fourcc(*'XVID')
    video_overlay = cv2.VideoWriter("%s_overlay.avi" % (video_file), video_type, 20.0, (320, 240))

    # make a timer for FPS computation
    start = time.time()
    end = 0
    try:
        i = 0
        while cap.isOpened():

            # test on 'i' frames
            if i==10:
                end = time.time()
                elapsedSeconds = end - start
                # print FPS
                print("Time taken : {0} seconds".format(elapsedSeconds))
                fps = i/elapsedSeconds
                print("Estimated FPS: {0}".format(fps))
                break

            _, frame = cap.read()
            print('frame %s' % i )
            combo_image= lane_follower.follow_lane(frame)
            cv2.imwrite("%s_%03d_%03d.png" % (video_file, i, lane_follower.curr_steering_angle), frame)
            cv2.imwrite("%s_overlay_%03d.png" % (video_file, i), combo_image)
            video_overlay.write(combo_image)
            cv2.imshow("Road with Lane line", combo_image)
            i += 1

            # press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # print overall FPS
                end = time.time()
                elapsedSeconds = end - start
                print("Time taken : {0} seconds".format(elapsedSeconds))
                fps = i/elapsedSeconds
                print("Estimated FPS: {0}".format(fps))
                break
    finally:
        cap.release()
        video_overlay.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)

    e1 = cv2.getTickCount()
# your code execution
    test_video('/home/pi/DeepPiCar/driver/data/tmp/video01')
    e2 = cv2.getTickCount()
    time = (e2 - e1)/ cv2.getTickFrequency()
    print("totaltime :", time)
