import cv2, os, numpy as np

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None: images.append(img)
    return images

boxes=dict()
boxes[0]=[240,210,230,160] #вручную указываем начальную детекцию дрона для первого кадра

drone_size = 0.5
k_dist=0.62
focal = 606 # 640x480
k_size = drone_size * focal

# NEW
detector = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)
# detector = cv2.ORB_create()
LK_PARAMS = dict(winSize=(30, 30), maxLevel=4,
                 criteria=(cv2.TermCriteria_EPS | cv2.TERM_CRITERIA_COUNT, 25, 0.1))
# NEW
def get_shifted_point(detector, img1, img2, x, y, dist):
    image_1, image_2 = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY), cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)
    keypoints_1_full = detector.detect(image_1)
    keypoints_1 = np.array([x.pt for x in keypoints_1_full], dtype=np.float32)
    keypoints_2, status, error = cv2.calcOpticalFlowPyrLK(image_1, image_2, keypoints_1, None, **LK_PARAMS)
    status = status.reshape(status.shape[0])
    keypoints_1, keypoints_2 = keypoints_1[status == 1], keypoints_2[status == 1]
    E, mask_match = cv2.findEssentialMat(keypoints_1, keypoints_2, method=cv2.RANSAC, prob=0.999, threshold=0.0003)
    if E is None: return None
    _, RotMatrix, d_xyz, mask = cv2.recoverPose(E, keypoints_1, keypoints_2)
    # Переносим точку из первого кадра во второй:
    point_3D_transformed = RotMatrix @ [[x], [y], [dist]] + d_xyz  # 3x1
    return point_3D_transformed

def main():
    imgs=load_images_from_folder("frames/")
    ind=-1

    last_x, last_y, last_d=0,0,0 #NEW
    last_dx, last_dy=0,0 #NEW
    prev_frame=None #NEW

    while True:
        ind=(ind+1)%len(imgs)
        frame = imgs[ind].copy()

        if ind>0:
            prev_frame=imgs[ind-1].copy()

        if ind in boxes.keys(): #1 детекция
            #get detection
            bbox=boxes[ind]
            x,y= bbox[0], bbox[1]
            drone_cx, drone_cy, w, h= x, y, bbox[2], bbox[3]
            x_, y_ = x - w // 2, y - h // 2
            drone_pix_sz = (w + h) / 2  # берем среднее от ширины и высоты
            drone_dist = k_dist * k_size / drone_pix_sz  # расчитываем
            print(f"d={drone_dist}")
            last_dx, last_dy=drone_cx-last_x, drone_cy-last_y #NEW
            last_x, last_y, last_d, last_bb  = drone_cx, drone_cy, drone_dist, bbox #NEW
            #draw
            label = f"d={drone_dist:.2f}"
            cv2.rectangle(frame, (x_, y_), (x_+w, y_+h), (0, 255, 0), 2)
            cv2.rectangle(frame, (drone_cx-2, drone_cy-2), (drone_cx+2, drone_cy+2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x_, y_ - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        elif prev_frame is not None and last_d>0: #2 трекинг  #NEW
            p=get_shifted_point(detector, prev_frame, frame, last_x, last_y, last_d)
            if p is not None:
                p=p.ravel()
                print(f"p={p}")
                x, y = int(p[0]),int(p[1])
                drone_cx, drone_cy, w, h = x, y, bbox[2], bbox[3]
                x_, y_ = x - w // 2, y - h // 2
                cv2.rectangle(frame, (x_, y_), (x_ + w, y_ + h), (255, 255, 0), 2)
                cv2.rectangle(frame, (drone_cx - 2, drone_cy - 2), (drone_cx + 2, drone_cy + 2), (255, 255, 0), 2)
                last_dx, last_dy = drone_cx - last_x, drone_cy - last_y
                last_x, last_y, last_d = x, y, last_d
            else: #3 экстраполяция  #NEW
                x, y = int(last_x+last_dx), int(last_y+last_dy)
                drone_cx, drone_cy, w, h = x, y, bbox[2], bbox[3]
                x_, y_ = x - w // 2, y - h // 2
                cv2.rectangle(frame, (x_, y_), (x_ + w, y_ + h), (0, 255, 255), 2)
                cv2.rectangle(frame, (drone_cx - 2, drone_cy - 2), (drone_cx + 2, drone_cy + 2), (0, 255, 255), 2)
                last_x, last_y, last_d = x, y, last_d

        cv2.imshow("output", frame)
        cv2.imwrite(f"{ind}.jpg", frame)
        if cv2.waitKey(1500) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
