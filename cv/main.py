import cv2
from pupil_apriltags import Detector  # windows是pupil_apriltags,linux就是apriltags


def detect_apriltag(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print("无法读取图片")
        return

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = Detector(families="tag36h11")  # 指定tag36h11家族
    results = detector.detect(gray)

    if not results:
        print("未检测到AprilTag标记")
        return

    for r in results:
        tag_id = r.tag_id
        center = tuple(r.center.astype(int))
        corners = r.corners.astype(int)

        # 绘制边界框和中心点
        cv2.polylines(image, [corners], True, (0, 255, 0), 2)
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            f"ID: {tag_id}",
            center,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )

        print(f"检测到标签ID: {tag_id}")
        print(f"中心坐标: {center}")
        print(f"角点坐标:\n{corners}\n")

    cv2.imshow("AprilTag Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_apriltag("test2.jpg")
