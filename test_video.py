import numpy as np 
import cv2
import argparse
import os

from visual_odometry import PinholeCamera, VisualOdometry


#skip = 3

traj = np.zeros((1200,1200,3), dtype=np.uint8)

def main():
	parser = argparse.ArgumentParser(prog='test_video.py', description='visual odometry script in python')
	parser.add_argument('--type', type=str, default='KITTI', choices=['KITTI','image','video'], metavar='<type of input>')
	parser.add_argument('--source', type=str, metavar='<path to source>')
	parser.add_argument('--camera_width', type=float, metavar='resolution in x direction')
	parser.add_argument('--camera_height', type=float, metavar='resolution in y direction')
	parser.add_argument('--focal', type=float, metavar='focal length in pixels')
	parser.add_argument('--pp_x', type=float, metavar='x coordinate of pricipal point')
	parser.add_argument('--pp_y', type=float, metavar='y coordinate of pricipal point')
	parser.add_argument('--skip', type=int, default=1)
	parser.add_argument('--verbose', type=bool, default=True, metavar='information of computing E')
	parser.add_argument('--show', type=bool, default=False, metavar='show the odometry result in real-time')
	args = parser.parse_args()
	print(args)
	if args.type == 'KITTI':
		cam = PinholeCamera(1241.0, 376.0, 718.8560, 718.8560, 607.1928, 185.2157)
		vo = VisualOdometry(cam, 'KITTI', '/home/r05525060/sharedfolder/indoorirPano/Dataset/dataset/poses/00.txt')
		for img_id in xrange(4541):
			img = cv2.imread('/home/r05525060/sharedfolder/indoorirPano/Dataset/dataset/sequences/00/image_0/{:06d}.png'.format(img_id), 0)

			vo.update(img, img_id)

			cur_t = vo.cur_t
			if(img_id > 2):
				x, y, z = cur_t[0], cur_t[1], cur_t[2]
			else:
				x, y, z = 0., 0., 0.
			draw_x, draw_y = int(x)+290, int(z)+90
			true_x, true_y = int(vo.trueX)+290, int(vo.trueZ)+90

			cv2.circle(traj, (draw_x,draw_y), 1, (img_id*255/4540,255-img_id*255/4540,0), 1) #TODO: 4540 to be changed
			cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
			cv2.rectangle(traj, (10, 20), (600, 60), (0,0,0), -1)
			text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
			cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)

			cv2.imshow('Road facing camera', img)
			cv2.imshow('Trajectory', traj)
			cv2.waitKey(1)

	if args.type == 'video':
		cap = cv2.VideoCapture(args.source)
		cam = PinholeCamera(args.camera_width, args.camera_height, args.focal, args.focal, args.pp_x, args.pp_y)
		vo = VisualOdometry(cam, type='video', annotations=None)
		frame_idx = 0
		frame_count = 0
		while(True):
			ret, frame = cap.read()
			frame_count += 1
			if not ret:
				print 'Video finished!'
				break
			if not frame_count % args.skip == 0:
				continue
			print 'processing... frame_count:', frame_count
			print 'processing... frame_index:', frame_idx
			gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			# print("shape of gray img:", gray_img.shape)
			vo.update(gray_img, frame_idx)
			cur_t = vo.cur_t
			if(frame_idx > 2):
				x, y, z = cur_t[0], cur_t[1], cur_t[2]
			else:
				x, y, z = 0., 0., 0.
			draw_x, draw_y = int(x)+590, int(z)+290
			true_x, true_y = int(vo.trueX)+590, int(vo.trueZ)+290

			# cv2.circle(traj, (draw_x,draw_y), 1, (frame_idx*255/4540,255-frame_idx*255/4540,0), 1)
			cv2.circle(traj, (draw_x,draw_y), 1, (0, 255 ,0), 2)
			cv2.circle(traj, (true_x,true_y), 1, (0,0,255), 2)
			cv2.rectangle(traj, (10, 20), (1200, 60), (0,0,0), -1) # black backgroud for text
			text = "Coordinates: x=%2fm y=%2fm z=%2fm"%(x,y,z)
			cv2.putText(traj, text, (20,40), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255), 1, 8)
			if args.show:
				cv2.imshow('Road facing camera', gray_img)
				cv2.imshow('Trajectory', traj)
				cv2.waitKey(1)
			frame_idx += 1
	cv2.putText(traj, "fps: {}.".format(30/args.skip), (20,100), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2, 8)
	cv2.putText(traj, "focal length: {} pixels".format(args.focal), (20,140), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,255), 2, 8)
	if args.type == "KITTI":
		cv2.imwrite('KITTI_map.jpg', traj)
	elif args.type == "video":
		cv2.imwrite('output_skip{}/{}_map_f{}_pp{}-{}.jpg'.format(args.skip, os.path.basename(args.source), args.focal, args.pp_x, args.pp_y), traj)


if __name__ == "__main__":
	main()