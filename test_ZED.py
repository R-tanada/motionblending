import pyzed.sl as sl
import cv2

def main():
    # ZEDカメラの初期化
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30

    zed = sl.Camera()
    if not zed.is_opened():
        print("ZEDカメラをオープン中...")
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("ZEDカメラをオープンできませんでした")
        exit()

    runtime_parameters = sl.RuntimeParameters()
    mat_left = sl.Mat()
    mat_right = sl.Mat()

    # メインループ
    key = ''
    while key != 113:  # 'q'キーで終了
        # ZEDカメラからフレームを取得
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # 左目の映像を取得
            zed.retrieve_image(mat_left, sl.VIEW.LEFT)
            left_image = mat_left.get_data()
            cv2.imshow("Left Image", left_image)

            # 右目の映像を取得
            zed.retrieve_image(mat_right, sl.VIEW.RIGHT)
            right_image = mat_right.get_data()
            cv2.imshow("Right Image", right_image)

        # キー入力をチェック
        key = cv2.waitKey(1)

    # ZEDカメラを閉じる
    zed.close()

if __name__ == "__main__":
    main()
