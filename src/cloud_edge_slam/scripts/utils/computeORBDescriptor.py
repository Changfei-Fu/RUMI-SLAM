
'''
        //This is for orientation
        // pre-compute the end of a row in a circular patch
        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
'''

'''
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
        {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
'''

'''
    static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
    {
        int m_01 = 0, m_10 = 0;

        const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int)image.step1();
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v*step], val_minus = center[u - v*step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return fastAtan2((float)m_01, (float)m_10);
    }
'''

import cv2
import math
import numpy as np


class ORBAngleComputer(object):
    HALF_PATCH_SIZE = 15
    def __init__(self):
        self.umax = self.ComputeUmax()
        pass

    def ComputeUmax(self):
        # calculate umax
        umax = np.zeros(shape=(self.HALF_PATCH_SIZE + 1,), dtype=np.int32)

        v = None
        v0 = None
        vmax = np.floor(self.HALF_PATCH_SIZE * math.sqrt(2.0) / 2 + 1)
        vmin = np.ceil(self.HALF_PATCH_SIZE * math.sqrt(2.0) / 2 + 1)
        hp2 = self.HALF_PATCH_SIZE ** 2
        v = 0
        while v <= vmax:
            umax[v] = np.round(math.sqrt(hp2 - v * v))
            v += 1
        # Make sure we are symmetric
        v = self.HALF_PATCH_SIZE
        v0 = 0
        while v >= vmin:
            while umax[v0] == umax[v0 + 1]:
                v0 += 1
            umax[v] = v0
            v0 += 1
            v -= 1
        return umax

    def Compute(self, image, pt_x, pt_y):
        m_01 = 0
        m_10 = 0

        u = -self.HALF_PATCH_SIZE
        while u <= self.HALF_PATCH_SIZE:
            m_10 += (u * image[pt_y, pt_x+u])
            u += 1

        v = 1
        while v <= self.HALF_PATCH_SIZE:
            v_sum = 0
            d = self.umax[v]
            u = -d
            while u <= d:
                val_plus = int(image[pt_y+v, pt_x+u])
                val_minus = int(image[pt_y-v, pt_x+u])
                v_sum += (val_plus - val_minus)
                m_10 += u * (val_plus + val_minus)
                u += 1
            m_01 += v * v_sum
            v += 1

        return cv2.fastAtan2(float(m_01), float(m_10))


if __name__ == '__main__':
    image_path = "/home/red0orange/图片/test.png"
    image = cv2.imread(image_path, 0)

    orb_angle_computer = ORBAngleComputer()

    for i in np.arange(100, 1820, 1820 / 10):
        for j in np.arange(100, 1100, 1100 / 10):
            i, j = map(int, [i, j])
            print("{} {}: ".format(i, j), orb_angle_computer.Compute(image, i, j))
    pass