import pyzed.sl as sl
import cv2

def main():
    # ZEDの初期化
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # 深度モードの設定
    zed = sl.Camera()
    if not zed.open(init_params) == sl.ERROR_CODE.SUCCESS:
        print("ZEDの初期化に失敗しました。")
        exit()

    # カメラパラメータの取得
    camera_info = zed.get_camera_information()
    image_size = camera_info.camera_resolution

    # 深度マップの設定
    depth_map = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.F32_C1)

    # ウィンドウの作成
    cv2.namedWindow("ZED Depth Map", cv2.WINDOW_NORMAL)

    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # 深度マップの取得
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

            # 深度マップの表示
            depth_data = depth_map.get_data()
            depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            cv2.imshow("ZED Depth Map", depth_image)

        # キー入力の受け付け
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # 終了処理
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()
