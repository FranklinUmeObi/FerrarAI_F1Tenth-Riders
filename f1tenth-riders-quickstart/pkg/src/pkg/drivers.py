import numpy as np

#------------------------------------------------------------------------
#Our Drivers Go here
#------------------------------------------------------------------------

class Frank:
    #Core variables
    baseSpeed = 11
    distToWallTrigger = 3.1
    turnStrength = 0.1
    brakeDist = distToWallTrigger*1.2

    def process_lidar(self, ranges):
        #Init Outputs
        speed = self.baseSpeed
        steering_angle = 0.0
        #AI
        if(ranges[440] < self.distToWallTrigger):    #turn left
            speed = self.baseSpeed * 0.65
            steering_angle = self.turnStrength

        if(ranges[640] < self.distToWallTrigger):    #turn right
            speed = self.baseSpeed * 0.65
            steering_angle =  -self.turnStrength

        if(ranges[540] < self.brakeDist or 
           ranges[520] < self.brakeDist or 
           ranges[560] < self.brakeDist):    #Don't crash
            speed = self.baseSpeed * 0.1
            
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------

class Sean:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------


class Sam:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------


class Chris:
    
    CAR_WIDTH = 0.31
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 2.
    SPEED = 13. 
    MAX_SPEED = 13.5
    # the extra safety room we plan for along walls (as a percentage of car_width/2)
    SAFETY_PERCENTAGE = 300.

    #below 4 funcs tell if car is on straight, can use this to accel faster here
    def getLeftAngle(self, ranges):
            third = int(len(ranges)/3)
            leftIndexes = ranges[0:third]

            #calculate lhs using full pi/2 lhs scanner range
            lhsI2Angle = ((len(leftIndexes)/2)/len(leftIndexes))*(np.pi/2) #for index 134 (of 270, maxIndex = 269)
            lhsI3Angle = np.pi/2 - lhsI2Angle # pi/2 window minus other angle in it

            lhsInnerAngleB = m.asin((leftIndexes[269]*m.sin(lhsI3Angle))
                /(m.sqrt((leftIndexes[269]*leftIndexes[269])+(leftIndexes[134]*leftIndexes[134]) 
                - (2*leftIndexes[269]*leftIndexes[134]*m.cos(lhsI3Angle)))))
            
            lhsInnerAngleA = m.asin((leftIndexes[0]*m.sin(lhsI2Angle))
                /(m.sqrt((leftIndexes[0]*leftIndexes[0])+(leftIndexes[134]*leftIndexes[134]) 
                - (2*leftIndexes[0]*leftIndexes[134]*m.cos(lhsI2Angle)))))

            lhsInnerAngleSum = lhsInnerAngleA+lhsInnerAngleB

            #print("L", lhsInnerAngleSum)
            return lhsInnerAngleSum

    def getRightAngle(self, ranges):
        third = int(len(ranges)/3)
        rightIndexes = ranges[(2*third):len(ranges)]
        #calculate RHS using full pi/2 RHS scanner range
        #RHS index 0 angle = 0
        rhsI2Angle = ((len(rightIndexes)/2)/len(rightIndexes))*(np.pi/2) #for index 134 (of 270, maxIndex = 269)
        rhsI3Angle = np.pi/2 - rhsI2Angle # pi/2 window minus other angle in it

        rhsInnerAngleB = m.asin((rightIndexes[269]*m.sin(rhsI3Angle))
            /(m.sqrt((rightIndexes[269]*rightIndexes[269])+(rightIndexes[134]*rightIndexes[134]) 
            - (2*rightIndexes[269]*rightIndexes[134]*m.cos(rhsI3Angle)))))
        
        rhsInnerAngleA = m.asin((rightIndexes[0]*m.sin(rhsI2Angle))
            /(m.sqrt((rightIndexes[0]*rightIndexes[0])+(rightIndexes[134]*rightIndexes[134]) 
            - (2*rightIndexes[0]*rightIndexes[134]*m.cos(rhsI2Angle)))))

        rhsInnerAngleSum = rhsInnerAngleA+rhsInnerAngleB #when its a straight this gets around 2.6 idk why, should be 3.14 (pi)
        #print("R", rhsInnerAngleSum)
        #end of rhs calculations
        return rhsInnerAngleSum

    def frontIsClear(self, proc_ranges):
        third = int(len(proc_ranges)/3)
        forwardIndexes = proc_ranges [third:(2*third)]
        isFrontClear = -1
        #index 134 is directly ahead, 104 is 10 degrees left of centre, 154 was meant
        #to be 164 to be 10 degrees to the right but this works so im not changing it haha
        sumof3 = forwardIndexes[104]+forwardIndexes[134]+forwardIndexes[154]
        if(sumof3 > 65): #when it's on a straight the sum of the 3 angles is above 65
            isFrontClear =1
        return isFrontClear

    def isOnStraight(self, proc_ranges, rightAngle, leftAngle):
        onStraight = -1
        if ((leftAngle < 2.72 and leftAngle > 2.5)and(rightAngle < 2.55 or rightAngle >2.35)
        and(self.frontIsClear(proc_ranges)>0)):
            print('On Straight ','L= ', leftAngle,', R= ', rightAngle)
            onStraight = 1
        return onStraight


    
    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges)/8)
        return np.array(ranges[eighth:-eighth])
    
     
    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.] # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i]-ranges[i-1]))
        return differences
    
    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2*np.arcsin(width/(2*dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx+1+i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx-1-i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        width_to_cover = (car_width/2) * (1+extra_pct/100)
        for index in disparities:
            first_idx = index-1
            points = ranges[first_idx:first_idx+2]
            close_idx = first_idx+np.argmin(points)
            far_idx = first_idx+np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist,
                    width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx,
                cover_right, ranges)
        return ranges
            
    def get_steering_angle(self, range_index, range_len,ranges):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """

        # if(ranges.argmin()>35):
        #     steering_angle = 0
        # else:
        if range_index < 100 or range_index>980:
            lidar_angle = (range_index - (range_len/2)) * self.radians_per_point
            print(lidar_angle)
            steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))*0.8
        elif (ranges[int(len(ranges)/2)] < 5):
            lidar_angle = (range_index - (range_len/2)) * self.radians_per_point*0.6
            # print(lidar_angle)
            steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        elif (ranges[int(len(ranges)/2)] < 10):
            lidar_angle = (range_index - (range_len/2)) * self.radians_per_point*0.3
            # print(lidar_angle)
            steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        else:
            lidar_angle = (range_index - (range_len/2)) * self.radians_per_point *0.2
            # print(lidar_angle)
            steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))

        return steering_angle
    
    def get_speed(self, ranges, max_disp,min_disp,proc_ranges,speed):
        # speed = 12
        # if (ranges[max_disp] - ranges[min_disp]) < 0.5:
        #     speed = speed * 0.2
        # if max_disp < 100 or max_disp>980:
        #     speed = speed * 0.2
        # if ranges[max_disp] < 3:
        #     speed = speed *0.2
        test = self.isOnStraight(proc_ranges, self.getRightAngle(proc_ranges), self.getLeftAngle(proc_ranges))
        if (ranges[int(len(ranges)/2)] < 6) and (speed > 3) and (ranges[int(len(ranges)/2)+2] < 6) and (ranges[int(len(ranges)/2)-2] < 6):
            speed = speed * 0.6
        elif (ranges[int(len(ranges)/2)] < 10) and (speed > 9):
            speed = speed * 0.7
        elif (ranges[int(len(ranges)/2)] < 12) and (speed > 10):
            speed = 10
        else:
            if speed < 8:
                speed = speed * 1.1
            if speed < 10:
                speed = speed * 1.5
            else:
                speed = self.MAX_SPEED

        return speed
    def process_lidar(self, ranges):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        self.radians_per_point = (2*np.pi)/len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                len(proc_ranges),ranges)
        
        speed = self.get_speed(ranges,proc_ranges.argmax(),proc_ranges.argmin(),proc_ranges,self.SPEED)
        self.SPEED = speed
        print(self.SPEED)
        # if starp_corner
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------


class Cormac:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------



#------------------------------------------------------------------------
#The Quick start Drivers
#------------------------------------------------------------------------
class GapFollower:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 3
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 8.0
    CORNERS_SPEED = 5.0
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self):
        # used when calculating the angles of the LiDAR data
        self.radians_per_elem = None

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # we won't use the LiDAR data from directly behind us
        proc_ranges = np.array(ranges[135:-135])
        # sets each value to the mean over a given window
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
            free_space_ranges: list of LiDAR data which contains a 'bubble' of zeros
        """
        # mask the bubble
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        # get a slice for each contigous sequence of non-bubble data
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        # I think we will only ever have a maximum of 2 slices but will handle an
        # indefinitely sized list for portablility
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        # do a sliding window average over the data in the max gap, this will
        # help the car to avoid hitting corners
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        """ Get the angle of a particular element in the LiDAR data and transform it into an appropriate steering angle
        """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

    def process_lidar(self, ranges):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(ranges)
        # Find closest point to LiDAR
        closest = proc_ranges.argmin()

        # Eliminate all points inside 'bubble' (set them to zero)
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        # Find max length gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Publish Drive message
        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------






#------------------------------------------------------------------------
#------------------------------------------------------------------------
# drives straight ahead at a speed of 5
class SimpleDriver:

    def process_lidar(self, ranges):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle


# drives toward the furthest point it sees
class AnotherDriver:

    def process_lidar(self, ranges):
        # the number of LiDAR points
        NUM_RANGES = len(ranges)
        # angle between each LiDAR point
        ANGLE_BETWEEN = 2 * np.pi / NUM_RANGES
        # number of points in each quadrant
        NUM_PER_QUADRANT = NUM_RANGES // 4

        # the index of the furthest LiDAR point (ignoring the points behind the car)
        max_idx = np.argmax(ranges[NUM_PER_QUADRANT:-NUM_PER_QUADRANT]) + NUM_PER_QUADRANT
        # some math to get the steering angle to correspond to the chosen LiDAR point
        steering_angle = max_idx * ANGLE_BETWEEN - (NUM_RANGES // 2) * ANGLE_BETWEEN
        speed = 5.0

        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------


import numpy as np


class DisparityExtender:
    
    CAR_WIDTH = 0.31
    # the min difference between adjacent LiDAR points for us to call them disparate
    DIFFERENCE_THRESHOLD = 2.
    SPEED = 5. 
    # the extra safety room we plan for along walls (as a percentage of car_width/2)
    SAFETY_PERCENTAGE = 300.

    def preprocess_lidar(self, ranges):
        """ Any preprocessing of the LiDAR data can be done in this function.
            Possible Improvements: smoothing of outliers in the data and placing
            a cap on the maximum distance a point can be.
        """
        # remove quadrant of LiDAR directly behind us
        eighth = int(len(ranges)/8)
        return np.array(ranges[eighth:-eighth])
    
     
    def get_differences(self, ranges):
        """ Gets the absolute difference between adjacent elements in
            in the LiDAR data and returns them in an array.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        differences = [0.] # set first element to 0
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i]-ranges[i-1]))
        return differences
    
    def get_disparities(self, differences, threshold):
        """ Gets the indexes of the LiDAR points that were greatly
            different to their adjacent point.
            Possible Improvements: replace for loop with numpy array arithmetic
        """
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        """ Returns the number of LiDAR points that correspond to a width at
            a given distance.
            We calculate the angle that would span the width at this distance,
            then convert this angle to the number of LiDAR points that
            span this angle.
            Current math for angle:
                sin(angle/2) = (w/2)/d) = w/2d
                angle/2 = sininv(w/2d)
                angle = 2sininv(w/2d)
                where w is the width to cover, and d is the distance to the close
                point.
            Possible Improvements: use a different method to calculate the angle
        """
        angle = 2*np.arcsin(width/(2*dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        """ 'covers' a number of LiDAR points with the distance of a closer
            LiDAR point, to avoid us crashing with the corner of the car.
            num_points: the number of points to cover
            start_idx: the LiDAR point we are using as our distance
            cover_right: True/False, decides whether we cover the points to
                         right or to the left of start_idx
            ranges: the LiDAR points

            Possible improvements: reduce this function to fewer lines
        """
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx+1+i
                if next_idx >= len(ranges): break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx-1-i
                if next_idx < 0: break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        """ For each pair of points we have decided have a large difference
            between them, we choose which side to cover (the opposite to
            the closer point), call the cover function, and return the
            resultant covered array.
            Possible Improvements: reduce to fewer lines
        """
        width_to_cover = (car_width/2) * (1+extra_pct/100)
        for index in disparities:
            first_idx = index-1
            points = ranges[first_idx:first_idx+2]
            close_idx = first_idx+np.argmin(points)
            far_idx = first_idx+np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist,
                    width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx,
                cover_right, ranges)
        return ranges
            
    def get_steering_angle(self, range_index, range_len):
        """ Calculate the angle that corresponds to a given LiDAR point and
            process it into a steering angle.
            Possible improvements: smoothing of aggressive steering angles
        """
        lidar_angle = (range_index - (range_len/2)) * self.radians_per_point
        steering_angle = np.clip(lidar_angle, np.radians(-90), np.radians(90))
        return steering_angle

    def process_lidar(self, ranges):
        """ Run the disparity extender algorithm!
            Possible improvements: varying the speed based on the
            steering angle or the distance to the farthest point.
        """
        self.radians_per_point = (2*np.pi)/len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges,
                self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        steering_angle = self.get_steering_angle(proc_ranges.argmax(),
                len(proc_ranges))
        speed = self.SPEED
        return speed, steering_angle
#------------------------------------------------------------------------
#------------------------------------------------------------------------
