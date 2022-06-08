#!/usr/bin/env python
# coding=utf-8
import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int64MultiArray, Int64, Bool
from cv_bridge import CvBridge, CvBridgeError


'''
領域を用いたトラッキング(最大面積を用いたcolor_tracking))
'''

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/pylon_camera_node/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/surveillance_camera_node/image_raw", Image, self.image_callback)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.mono_image = np.arange(27).reshape(3, 3, 3)

    def image_callback(self, image_data):
        try:
            self.mono_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


class ColorTracking:
    COLOR = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    # 画像の縦横
    IMAGE_HEIGHT = 480  #1200
    IMAGE_WIDTH = 720   #1920
    # 領域数
    REGION_NUMBER = 1
    # 四角形の枠の半径
    FRAME_SIZE = 10
    # TODO: ここはあとでrename
    # 切断時にずらすピクセル
    # SHIFT_START_POINT = 0
    SHIFT_START_POINT = 10
    # SHIFT_END_POINT = 80
    SHIFT_END_POINT = 150
    # カメラと測定場所の距離[mm]
    BASE_LENGTH = 1500
    # 測定場所でのTの高さ[ピクセル]
    # BASE_HEIGHT = 106
    # BASE_HEIGHT = 115
    # BASE_HEIGHT = 125
    # BASE_HEIGHT = 135
    BASE_HEIGHT = 82
    # Tの高さ[mm]
    T_HEIGHT = 105
    # # 画面の幅
    IMAGE_CENTER_X = 360  #960
    # 画面の高さ
    IMAGE_CENTER_Y = 240  #600
    # モルフォロジー演算を使用するときのカーネルサイズ TODO:MORPHOLOGY_KERNEの1stと2ndを現地調整
    MORPHOLOGY_KERNEL_CLOSE_1st = np.ones((20,20),np.uint8)
    # MORPHOLOGY_KERNEL_OPEN = np.ones((2,2),np.uint8)
    # MORPHOLOGY_KERNEL_2nd = np.ones((60,60),np.uint8)
    # MORPHOLOGY_KERNEL_2nd = np.ones((70,70),np.uint8)
    # MORPHOLOGY_KERNEL_CLOSE_2nd = np.ones((80,80),np.uint8)
    # MORPHOLOGY_KERNEL_CLOSE_2nd = np.ones((100,100),np.uint8)
    # MORPHOLOGY_KERNEL_2nd = np.ones((160,160),np.uint8)



    def __init__(self):
        # TODO:ここは現地で値の変更を行う
        # カーネルポイント
        self.mask_point = 5
        # 収縮用のカーネルの作成
        self.contraction_kernel = np.ones((self.mask_point, self.mask_point), np.uint8)
        # 膨張用のカーネルの作成
        self.dilatation_kernel = np.ones((self.mask_point + 10, self.mask_point + 10), np.uint8)
        # TODO: ここのインスタンス変数はどうするか検討
        self.region_image = np.arange(27).reshape(3, 3, 3)
        self.region_top_left_x = 0
        self.region_top_left_y = 0
        self.region_width = 0
        self.prev_region_width = 0
        self.region_height = 0
        self.prev_region_height = 0
        # robotの座標を扱う配列（int64型の配列に変換）
        self.robot_coordinate = Int64MultiArray()
        # TODO: change
        #[0,0,0,0]の配列を作り他の変数に与えている
        self.robot_coordinate.data = [0] * 4
        self.prev_robot_coordinate = self.robot_coordinate
        self.bridge = CvBridge()
        self.is_autonomous = False
        self.robot_to_camera_x = 0
        self.robot_to_camera_y = 0
        self.robot_to_camera_z = 0
        self.moving_average = np.zeros(4)
        # TODO: debug
        self.debug = self.robot_coordinate
        self.debug_publish = rospy.Publisher('debug', Int64MultiArray, queue_size=1)
        # publisher
        self.tracking_image_pub = rospy.Publisher('tracking_image', Image, queue_size=1)
        self.robot_position_publish = rospy.Publisher('robot_position', Int64MultiArray, queue_size=1)
        self.yaw_axis_data_publish = rospy.Publisher('yaw_axis_data', Int64, queue_size=1)
        self.autonomous_mode_publish = rospy.Publisher('autonomous_mode', Bool, queue_size=1)
        self.depth_publish=rospy.Publisher('depth_data',Int64,queue_size=1)
        # set_param
        rospy.set_param('is_autonomous', False)
        self.count = 0

    '''
    このプログラムのメイン
    '''
    def detectors_main(self, image):
        print (image.shape)
        if image.shape[0] <= 3:
            return False
        filterd_image = self.filter_image(image, self.contraction_kernel, self.dilatation_kernel)
        is_set_region = self.region_main(filterd_image)
        if not is_set_region:
            self.transmit_control_mode(is_autonomous=False)
            rospy.logerr("Don't recognize")
            return False
        # 切断する座標をここで決定
        top_surface_max_x = self.region_top_left_x + int(self.region_width) #+100
        top_surface_max_y = self.region_top_left_y + int(self.region_height / 3.0) #+100
        # top_surface_max_y = self.region_top_left_y + int(self.region_height / 5.0) #+100/
        # cv2.imshow('a', filterd_image)
        trimmed_top_image = filterd_image[self.region_top_left_y:top_surface_max_y, self.region_top_left_x:top_surface_max_x]
        trimmed_bottom_image = filterd_image[top_surface_max_y + self.SHIFT_START_POINT:top_surface_max_y + self.SHIFT_END_POINT,
                                             self.region_top_left_x:top_surface_max_x]
        # 切断面の上の画像
        top_image, _ = self.analyze_label_data(trimmed_top_image)
        top_center_coordinate = self.calculate_center_coordinate(top_image)
        if len(top_center_coordinate) < self.REGION_NUMBER:
            self.transmit_control_mode(is_autonomous=False)
            return False
        # cv2.imshow('trimmed_top_image', trimmed_top_image)
        
        top_center_x = top_center_coordinate[0][0] + self.region_top_left_x
        top_center_y = top_center_coordinate[0][1] + self.region_top_left_y
        # 下の画像
        bottom_image, _ = self.analyze_label_data(trimmed_bottom_image)

        bottom_center_coordinate = self.calculate_center_coordinate(bottom_image)
        if len(bottom_center_coordinate) < self.REGION_NUMBER:
            self.transmit_control_mode(is_autonomous=False)
            return False
        # cv2.imshow('trimmed_bottom_image', trimmed_bottom_image)

        # yaw軸のブレみるときにはx軸方向しかみないのでxのみ計算
        bottom_center_x = bottom_center_coordinate[0][0] + self.region_top_left_x
        center = bottom_center_coordinate[0] + self.region_top_left_x
        convert_x, convert_y, convert_z, depth = self.calculate_pixel_to_millimeter(float(self.region_height), center)
        rospy.logwarn(self.robot_coordinate.data[0] - convert_x)
        #バウンデングボックスの描画
        cv2.rectangle(image, (top_center_x - self.FRAME_SIZE, top_center_y - self.FRAME_SIZE),
                        (top_center_x + self.FRAME_SIZE, top_center_y + self.FRAME_SIZE), self.COLOR[0], 5)
        cv2.rectangle(image, (bottom_center_x - self.FRAME_SIZE, top_center_y - self.FRAME_SIZE),
                        (bottom_center_x + self.FRAME_SIZE, top_center_y + self.FRAME_SIZE), self.COLOR[2], 5)
        cv2.imshow('tracking_image', image)
        image_msg = self.bridge.cv2_to_imgmsg(image)
        rospy.set_param('/is_robot_lost', False)
        self.tracking_image_pub.publish(image_msg)
        self.publish_image(image)
        
        # if abs(self.prev_region_width - self.region_width) < 30:
        #     if abs(self.prev_region_height - self.region_height) > 30:
        #         return False
        #     else:
        #         self.prev_region_height = self.region_height
        # self.prev_region_width = self.region_width

        self.publish_robot_coordinate(top_center_x)
        self.publish_yaw_axis_data(bottom_center_x)
        
        self.count = 0
        return True

    '''
    以下領域を作る処理
    '''
    def region_main(self, filterd_image):
        labeling_image, labeling_data = self.analyze_label_data(filterd_image)
        # 最大面積のマーカーを検出
        region_data, center= self.decision_region(labeling_image, labeling_data)
        if len(labeling_data) < self.REGION_NUMBER:
            return False
        # 最大面積のあるところを領域にする
        top_left_x, top_left_y, width, height = self.blob_detection(labeling_image, region_data)
        # 領域の左上の座標、右下の座標をそれぞれ計算
        region_min_x, region_max_x, region_min_y, region_max_y, width, height = self.calculate_region(width, height, top_left_x, top_left_y)
        rospy.loginfo(height)
        # 領域のparameterをここでセット
        self.set_region_param(filterd_image, region_min_x, region_max_x, region_min_y, region_max_y, width, height)
        convert_x, convert_y, convert_z, depth = self.calculate_pixel_to_millimeter(float(height), center)
        self.publish_depth_data(depth)
        self.set_robot_to_camera(convert_x, convert_y, convert_z)
        return True

    def decision_region(self, labeling_image, labeling_data):
        area = labeling_data[:, 4]
        sort_index_num = self.area_index_asc(area)
        region_data = self.select_region_data(sort_index_num, labeling_data)
        center_coordinate = self.calculate_center_coordinate(labeling_image)
        if len(center_coordinate) < self.REGION_NUMBER:
            return region_data, center_coordinate
        center = self.select_region_data(sort_index_num, center_coordinate)[0]
        return region_data, center

    def area_index_asc(self, area):
        return np.argsort(area)[::-1]

    def select_region_data(self, indexes, labeling_data):
        data = [0] * len(indexes)
        for i, area_desc in enumerate(indexes):
            data[i] = labeling_data[area_desc]
        return np.array(data[:self.REGION_NUMBER])

    def calculate_region(self, width, height, x_data, y_data):
        base_width = width.max()
        base_height = height.max()
        print (base_height)
        rospy.logerr(base_height)
        region_min_x = x_data.min()
        region_max_x = x_data.min() + base_width
        region_min_y = y_data.min() + base_height
        region_max_y = y_data.max()
        region_min_x, region_max_x = self.limit_decision(region_min_x, 0, region_max_x, self.IMAGE_WIDTH)
        region_min_y, region_max_y = self.limit_decision(region_min_y, 0, region_max_y, self.IMAGE_HEIGHT)
        return region_min_x, region_max_x, region_min_y, region_max_y, base_width, base_height

    def limit_decision(self, region_min, limit_min, region_max, limit_max):
        # マーカーを手でかくした時にregion_max, region_min の大小関係がおかしくなることがあるのでとりあえず救済処置
        if region_max < region_min:
            region_min, region_max = self.change_data(region_min, region_max)
        # 領域外だったときの対処
        if region_min < limit_min:
            region_min = limit_min
        if region_max >= limit_max:
            region_max = limit_max
        return region_min, region_max

    def change_data(self, data_1, data_2):
        return data_2, data_1

    def set_region_param(self, image, region_min_x, region_max_x, region_min_y, region_max_y, width, height):
        self.region_image = image[region_min_y:region_max_y,
                                  region_min_x:region_max_x]
        # 領域の左上の座標
        self.region_top_left_x = region_min_x
        self.region_top_left_y = region_min_y
        self.region_width = width
        self.region_height = height

    def set_robot_to_camera(self, x, y, z):
        self.robot_coordinate.data[0] = x
        self.robot_coordinate.data[1] = y
        self.robot_coordinate.data[2] = z  
        
    '''
    以下ラベリング解析、ブロブ解析
    マスク映像の中から中心座標等を返す
    ラベリング
    画像中の連結した領域（connected components）を抽出する解析手法（画像処理）
    ブロブ解析
    ラべリングを行い、ラベル付けされた塊の面積、位置、長さなどの特徴量を解析する手法
    '''
    
    def analyze_label_data(self, image):
        labeling_image = cv2.connectedComponentsWithStats(image)
        #検出したオブジェクトの第三引数を取り出し、オブジェクトのサイズは使わないので削除
        labeling_data = np.delete(labeling_image[2], 0, 0).astype(np.int16) 
        return labeling_image, labeling_data

    def blob_detection(self, labeling_image, labeling_data):
        top_left_x = labeling_data[:, 0]
        top_left_y = labeling_data[:, 1]
        width = labeling_data[:, 2]
        height = labeling_data[:, 3]
        return top_left_x, top_left_y, width, height

    def calculate_center_coordinate(self, labeling_image):
        # 中心座標の解析
        center = np.delete(labeling_image[3], 0, 0).astype(np.int16)
        return center

    def calculate_pixel_to_millimeter(self, height, center):
        """
        ピクセルデータをミリメートルに変換した値を返す　TODO:参考URLを。。
        Inputs
        -------
        height : float
            Tの高さ
        center : tuple(int)
            中心座標
        
        Returns
        -------
        convert_x : float
            カメラから見たTのx座標[mm]
        convert_y : float
            カメラから見たTのy座標[mm]
        convert_z : float
            カメラから見たTのz座標[mm]
        depth : float FIXME:後でコメント修正
            カメラから見たTのx座標[mm]
        """
        #奥行き方向
        depth =self.BASE_HEIGHT / height * self.BASE_LENGTH
        #1ピクセル当たりの大きさを計算(ピクセルからmmに変換するための比率)
        convention_value = self.T_HEIGHT / height
        #三次元位置(X, Y, Z)を計算
        # print center

        convert_x = (center[0] - self.IMAGE_CENTER_X) * convention_value
        convert_y = (self.IMAGE_CENTER_Y - center[1]) * convention_value
        if depth > convert_x:
            convert_z = np.sqrt(depth * depth - convert_x * convert_x)
        else: 
            convert_z = depth
        convert_x, convert_y, convert_z, depth = round(convert_x), round(convert_y), round(convert_z), round(depth)
        print (convert_x, convert_y, convert_z, depth)
        return convert_x, convert_y, convert_z, depth

    def calculate_moving_average(self, robot_coordinate):
        self.moving_average = np.vstack((self.moving_average, robot_coordinate))
        if len(self.moving_average) > 15:
            self.moving_average = np.delete(self.moving_average, 0, axis=0)
            average = np.mean(self.moving_average, axis=0)
            return average
        return robot_coordinate

    '''
    以下マスク映像(特定の色だけ抽出して白と黒の2つの色しかない画像（２値化画像）)の取得
    膨張収縮処理を使って簡単なノイズ除去
    '''
    def filter_image(self, image, contraction, dilate):
        #imageの形の変換
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 設定した閾値で2値化
        _, extraction_image = cv2.threshold(gray_image, 240, 255, cv2.THRESH_BINARY)
        # cv2.imshow('1', extraction_image)
        # 膨張
        filterd_image = cv2.dilate(extraction_image, dilate)
        # クロージング処理は膨張後に収縮
        filterd_image = cv2.morphologyEx(extraction_image, cv2.MORPH_CLOSE, self.MORPHOLOGY_KERNEL_CLOSE_1st)
        cv2.imshow('filterd_image', filterd_image)
        # filterd_image = cv2.bitwise_xor(extraction_image, filterd_image)
        # cv2.imshow('3', filterd_image)
        # filterd_image = cv2.morphologyEx(filterd_image, cv2.MORPH_OPEN, self.MORPHOLOGY_KERNEL_OPEN)
        # cv2.imshow('4', filterd_image)
        # # img_copy = np.copy(filterd_image)
        # filterd_image = cv2.morphologyEx(filterd_image, cv2.MORPH_CLOSE, self.MORPHOLOGY_KERNEL_CLOSE_2nd)
        # cv2.imshow('filterd_image', filterd_image)

        # img_copy = cv2.morphologyEx(img_copy, cv2.MORPH_CLOSE, np.ones((80,80),np.uint8))
        # cv2.imshow('img_copy', img_copy)

        # filterd_image = cv2.dilate(filterd_image, dilate)
        # cv2.imshow('filterd_mask', filterd_image)
        return filterd_image

    '''
    以下publish
    '''
    def publish_image(self, image):
        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.tracking_image_pub.publish(image_msg)

    def publish_robot_coordinate(self, pixel_data_x):
        self.robot_coordinate.data[3] = pixel_data_x   #を入力
        self.debug_publish.publish(self.robot_coordinate)
        self.robot_coordinate.data = self.calculate_moving_average(self.robot_coordinate.data)   #TODO なぜ平均を出す
        self.robot_position_publish.publish(self.robot_coordinate)
        # self.robot_position_publish.publish(average)

    def publish_yaw_axis_data(self, yaw):
        self.yaw_axis_data_publish.publish(yaw)

    def publish_depth_data(self,depth):
        self.depth_publish.publish(depth)

    def transmit_control_mode(self, is_autonomous):
        is_control = rospy.get_param('is_autonomous', False)
        rospy.set_param('/is_robot_lost', True)
        if is_control and (self.count >= 45):
            self.autonomous_mode_publish.publish(is_autonomous)
            self.is_autonomous = is_autonomous
            rospy.set_param('is_autonomous', False)
            self.count = 0
            self.moving_average = np.zeros(4)
        self.count += 1


def main():
    rospy.init_node('MonoColorTracking')
    colortracking = ColorTracking()
    ic = ImageConverter()
    rate = rospy.Rate(15)
    cap = cv2.VideoCapture(-1)
    #カメラ画像を取得できない場合quitでプログラムを終了する
    if not cap.isOpened():
    	print('Failed to open camera')
    	quit()
    _, im = cap.read()
    while not rospy.is_shutdown():
        start = time.time()
        # イメージの読み込み
        cv2.waitKey(1)
        # cv2.imshow('image_raw', ic.mono_image)
        # is_tracking = colortracking.detectors_main(ic.mono_image)
        _, im = cap.read()
        cv2.namedWindow('image_raw', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('image_raw', im)
        is_tracking = colortracking.detectors_main(im)
        # 制御周期
        elapsed_time = time.time() - start
        print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")
        rate.sleep()
    cv2.destroyAllWindows()     # すべてのウィンドウを閉じる


if __name__ == '__main__':
    main()
