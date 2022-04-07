import numpy as np
import cv2, random, math, copy
import rospy

from liner import Liner

class SlidingLiner(Liner):
    def callback(self, msg):
        src = self.imgmsg2numpy(msg)
        self.width = msg.width
        self.height = msg.height

        warp_img_w = 320
        warp_img_h = 240

        warpx_margin = 50
        warpy_margin = 3

        warp_src_pts = np.array([[300 - warpx_margin, 370 - warpy_margin],
        [200 - warpx_margin, 550 + warpy_margin],
        [500 + warpx_margin, 370 - warpy_margin],
        [600 + warpx_margin, 550 + warpy_margin]], dtype=np.float32)


        warp_dist_pts = np.array([[0, 0],
        [0, warp_img_h],
        [warp_img_w, 0],
        [warp_img_w, warp_img_h]], dtype=np.float32)

        # calibration 을 제대로 해야 정상적인 결과가 나올듯하다.
        warp_img , mat, mat_inv = self.warp_image(src, warp_src_pts, warp_dist_pts, (warp_img_w, warp_img_h))
        lfit, rfit = self.warp_process_image(warp_img)
        lane_img = self.draw_lane(src, warp_img, mat_inv, lfit, rfit)

        cv2.imshow("lane_img", lane_img)
        #cv2.imshow("src", src)
        cv2.waitKey(10)

    def warp_image(self, img, src, dst, size):
        mat  = cv2.getPerspectiveTransform(src, dst)
        mat_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, mat, size, flags=cv2.INTER_LINEAR)
        cv2.imshow("warped", warp_img)
        return warp_img, mat, mat_inv

    def warp_process_image(self, img):
        n_windows = 9
        margin = 12
        minpix = 5

        blur = cv2.GaussianBlur(img, (0, 0), 1)
        lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)

        lower = np.array([150, 0, 150])
        upper = np.array([255, 145, 255])

        lane = cv2.inRange(lab, lower, upper)

        histogram = np.sum(lane[lane.shape[0]//2:, :], axis = 0)
        mid = np.int(histogram.shape[0]/2)

        leftx_current = np.argmax(histogram[:mid])
        rightx_current = np.argmax(histogram[mid:]) + mid
        window_height = np.int(lane.shape[0]/n_windows)
        nz = lane.nonzero()

        left_lane_idxs = []
        right_lane_idxs = []
        lx, ly, rx, ry = [], [], [], []

        hist = np.dstack((lane, lane, lane))*255

        for window in range(n_windows):
            win_yl = lane.shape[0] - (window + 1)*window_height
            win_yh = lane.shape[0] - window*window_height

            win_xll = leftx_current - margin
            win_xlh = leftx_current + margin
            win_xrl = rightx_current - margin
            win_xrh = rightx_current + margin

            cv2.rectangle(hist, (win_xll, win_yl), (win_xlh, win_yh), (0, 255, 0), 2)
            cv2.rectangle(hist, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 2)

            good_left_idxs = ((nz[0] >= win_yl) & (nz[0] >= win_yh) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
            good_right_idxs = ((nz[0] >= win_yl) & (nz[0] >= win_yh) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]

            left_lane_idxs.append(good_left_idxs)
            right_lane_idxs.append(good_right_idxs)

            if len(good_left_idxs) > minpix:
                leftx_current = np.int(np.mean(nz[1][good_left_idxs]))
            
            if len(good_right_idxs) > minpix:
                rightx_current = np.int(np.mean(nz[1][good_left_idxs]))

            lx.append(leftx_current)
            ly.append((win_yl + win_yh)/2)
            rx.append(rightx_current)
            ry.append((win_yl + win_yh)/2)

        left_lane_idxs = np.concatenate(left_lane_idxs)
        right_lane_idxs = np.concatenate(right_lane_idxs)

        lfit = np.polyfit(np.array(ly), np.array(lx), 2)
        rfit = np.polyfit(np.array(ry), np.array(rx), 2)

        hist[nz[0][left_lane_idxs], nz[1][left_lane_idxs]] = [255, 0, 0]
        hist[nz[0][right_lane_idxs], nz[1][right_lane_idxs]] = [0, 0, 255]

        cv2.imshow("hist", hist)

        return lfit, rfit

    def draw_lane(self, image, warp_img, mat_inv, lfit, rfit):
        y_max = warp_img.shape[0]
        ploty = np.linspace(0, y_max -1, y_max)
        color_warp = np.zeros_like(warp_img).astype(np.uint8)

        lfitx = lfit[0]*ploty**2 + lfit[1]*ploty + lfit[2]
        rfitx = rfit[0]*ploty**2 + rfit[1]*ploty + rfit[2]

        pts_left = np.array([np.transpose(np.vstack([lfitx, ploty]))]) 
        pts_right = np.array([np.flipud(np.transpose(np.vstack([rfitx, ploty])))])    
        pts = np.hstack((pts_left, pts_right))

        color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        new_warp = cv2.warpPerspective(color_warp, mat_inv, (self.width, self.height))

        return cv2.addWeighted(image, 1, new_warp, 0.3, 0)




sliding_liner = SlidingLiner('sliding_liner')
sliding_liner.run()