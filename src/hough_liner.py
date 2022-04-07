#!/usr/bin/env python

import rospy, random, cv2, math
import numpy as np

from liner import Liner

class HoughLiner(Liner):
   
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 30.0
    delay = round(1000/fps)
    out = cv2.VideoWriter('output.mp4', fourcc, fps, (640, 480))
    target_b = 128
    prev_angle = 0
    back = False
    back_count = 20

    def callback(self, msg):
        frame = self.imgmsg2numpy(msg)
        self.width_offset = 0
        self.width = msg.width
        self.height = msg.height
        self.offset = 310
        self.gap = 60
        self.lpos = self.width_offset
        self.rpos = self.width - self.width_offset
        
        if self.back:
            if self.back_count > 0:
                self.back_count -= 1
                self.controller.steer(self.prev_angle, self.back)
                self.out.write(frame)
                return
            else:
                self.back_count = 20
                self.back = False
        

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (0, 0), 2.0)

        low_thres = 60
        high_thres = 70
        roi = gray[self.offset:self.offset + self.gap, 0+self.width_offset:self.width-self.width_offset]
        curr_b = np.mean(roi)
        
        gray = gray + np.uint8(self.target_b - curr_b)

        ret, gray = cv2.threshold(gray, 124, 255, cv2.THRESH_BINARY_INV)
        #cv2.imshow("gray", gray)
        
        edge = cv2.Canny(np.uint8(gray), low_thres, high_thres)
        roi = edge[self.offset:self.offset + self.gap, 0+self.width_offset:self.width-self.width_offset]
        

        lines = cv2.HoughLinesP(roi, 1, math.pi/180, 30, 30, 10)

        mid = "None"
        angle = "None"
        if lines is not None:
            left_lines, right_lines, mid = self.divide_left_right(lines)
            frame, self.lpos = self.get_line_pos(frame, left_lines, left=True)
            frame, self.rpos = self.get_line_pos(frame, right_lines, right=True)

            frame = self.draw_lines(frame, left_lines)
            frame = self.draw_lines(frame, right_lines)
            frame = self.draw_rectangle(frame)
            
            if self.lpos == self.width_offset:
                if self.rpos > self.width*0.75:
                    angle = 0
                elif self.rpos > self.width*0.5:
                    angle = -500*((self.width*0.75 - self.rpos)/self.width)
                elif self.rpos > self.width*0.35:
                    angle = -50
                else:
                    angle = 50
                    self.back = True
            elif self.rpos == self.width - self.width_offset:
                if self.lpos < self.width*0.25:
                    angle = 0
                elif self.lpos < self.width*0.5:
                    angle = 500*((self.lpos - self.width*0.25)/self.width)
                elif self.lpos < self.width*0.65:
                    angle = 50
                else:
                    angle = -50
                    self.back = True
            else:                
                center = (self.lpos + self.rpos)/2
                error = (center - self.width/2)
                if abs(self.lpos - self.rpos) < 135 or self.rpos < self.lpos:
                    if self.prev_angle > 0:
                        angle = 50
                    elif self.prev_angle < 0:
                        angle = -50
                    else:
                        angle =0
                else:
                    print(error)
                    if error < 100:
                        angle = self.s_pid.pid_control(error)*1
                    else:
                        angle = self.c_pid.pid_control(error)*1

            self.prev_angle = angle
            self.controller.steer(angle, self.back)
            #print("lpos: {}, rpos: {}".format(self.lpos, self.rpos))
        else:
            self.out.release()
            exit()
         
        font=cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(frame, "angle " + str(angle), (50,100), font,1,(255,0,0),2)
        cv2.putText(frame, str(self.lpos) + ", " + str(self.rpos), (50,440), font,1,(255,0,0),2)
        cv2.putText(frame, "mid " +str(mid), (440, 50), font,1,(255,0,0),2)
        
        self.out.write(frame)

        if cv2.waitKey(10) == 27:
            self.out.release()
            exit()  

    def draw_lines(self, img, lines):
        for line in lines:
            x1, y1, x2, y2 = line
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1 + self.width_offset, y1 + self.offset), (x2 + self.width_offset, y2 + self.offset), color, 2)
        return img

    def draw_rectangle(self, img):
        ccen = (self.lpos + self.rpos)/2
        ocen = self.width/2

        cv2.rectangle(img, (self.lpos - 5, 15 + self.offset), (self.lpos + 5, 25 + self.offset), (0, 255, 0), 2)
        cv2.rectangle(img, (self.rpos - 5, 15 + self.offset), (self.rpos + 5, 25 + self.offset), (0, 255, 0), 2)
        cv2.rectangle(img, (ccen - 5, 15 + self.offset), (ccen + 5, 25 + self.offset), (0, 255, 0), 2)
        cv2.rectangle(img, (ocen - 5, 15 + self.offset), (ocen + 5, 25 + self.offset), (0, 0, 255), 2)

        return img

    def divide_left_right(self, lines):
        low_grad_thres = 0
        high_grad_thres = 10

        filtered_lines = []
        left_lines = []
        right_lines = []

        max_grad = -20
        min_grad = 20
        max_x = 0
        min_x = self.width
        negative = False
        positive = False


        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y2 - y1 == 0:
                grad = 0
            else:
                grad = float(x2 - x1)/float(y2 - y1)
            
            if (abs(grad) > low_grad_thres) and (abs(grad) < high_grad_thres):
                if max_grad < grad:
                    max_grad = grad
                if min_grad > grad:
                    min_grad = grad
                if x1 > max_x:
                    max_x = x1
                if x1 < min_x:
                    min_x = x1
                
                if grad >= 0:
                    positive = True
                else:
                    negative = True

                filtered_lines.append((line, grad))

        if max_x - min_x > 400:
            mid = (max_grad + min_grad)/2
        else:
            mid = 0
            positive = True
            negative = True
        #print(mid)
        
        for line, grad in filtered_lines:
            x1, y1, x2, y2 = line[0]
            if grad < mid:
                left_lines.append(line[0].tolist())
            else:
                right_lines.append(line[0].tolist())
            # if positive and negative:
            #     if grad > mid:
            #         left_lines.append(line[0].tolist())
            #     else:
            #         right_lines.append(line[0].tolist())
            # else:
            #     if grad < mid:
            #         left_lines.append(line[0].tolist())
                    
            #     else:
            #         right_lines.append(line[0].tolist())

        return left_lines, right_lines, mid

    def get_line_params(self, lines):
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if not size:
            return 0, 0
        
        for line in lines:
            x1, y1, x2, y2 = line

            m = float(y2 - y1)/float(x2 - x1)

            # if abs(m) < 0.5:
            #     size -= 1
            #     continue
        
            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += m

        x_mean = x_sum/(size*2)
        y_mean = y_sum/(size*2)
        m = m_sum/size
        b = y_mean - m * x_mean

        return m, b

    def get_line_pos(self, img, lines, left=False, right=False):
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0
            if right:
                pos = self.width
        else:
            y = self.gap/2
            pos = int((y - b)/m)

            b += self.offset
            x1 = (self.height - b)/float(m)
            x2 = ((self.height/2) - b)/float(m)

            cv2.line(img, (int(x1) + self.width_offset, self.height), (int(x2) + self.width_offset, self.height/2), (255, 0, 0), 3)

        return img, pos + self.width_offset

    def set_steering(self):
        pass

hough_liner = HoughLiner('hough_liner')
hough_liner.run()