import numpy as np
import cv2
import os
import time
from utils import frame_to_bgr_image
from pyorbbecsdk import *

class Gemini335:
    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        # 深度范围裁剪 (mm)，用于可视化效果
        self.min_depth = 150
        self.max_depth = 3000 
        
        self.pipeline = Pipeline()
        self.config = Config()
        
        # === 初始化对齐过滤器 ===
        try:
            self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        except Exception as e:
            print(f"创建对齐过滤器失败: {e}")
            self.align_filter = None

        try:
            self.setup_streams()
            # 尝试开启帧同步
            try:
                self.pipeline.enable_frame_sync()
            except:
                pass
            self.pipeline.start(self.config)
            print(f"Gemini335相机启动成功")
            self.get_camera_intrinsics()
            
        except Exception as e:
            print(f"相机启动失败: {e}")
            raise
    
    def setup_streams(self):
        # 1. 配置彩色流 (优先 MJPG)
        color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = None
        try:
            color_profile = color_profile_list.get_video_stream_profile(self.width, self.height, OBFormat.MJPG, self.fps)
        except:
            try:
                color_profile = color_profile_list.get_video_stream_profile(self.width, self.height, OBFormat.RGB, self.fps)
            except:
                color_profile = color_profile_list.get_default_video_stream_profile()

        self.config.enable_stream(color_profile)

        # 2. 配置深度流 (Y16)
        depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        depth_profile = None
        try:
            depth_profile = depth_profile_list.get_video_stream_profile(self.width, self.height, OBFormat.Y16, self.fps)
        except:
            depth_profile = depth_profile_list.get_default_video_stream_profile()
            
        self.config.enable_stream(depth_profile)

    def get_camera_intrinsics(self):
        try:
            camera_param = self.pipeline.get_camera_param()
            self.depth_intrinsics = camera_param.depth_intrinsic
            self.color_intrinsics = camera_param.rgb_intrinsic
            print(f"内参: fx={self.depth_intrinsics.fx:.1f}")
        except:
            pass

    def capture_frame(self):
        """
        采集单帧数据
        返回 4 个数据: Color, Raw Depth(uint16), Depth Viz(彩色热力图), Overlay
        """
        try:
            frames = self.pipeline.wait_for_frames(100)
            if frames is None: return None, None, None, None
            
            # 执行对齐
            if self.align_filter:
                frames = self.align_filter.process(frames)
                if frames is None: return None, None, None, None
                frames = frames.as_frame_set()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame: return None, None, None, None
            
            # 1. 处理彩色图
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None and color_frame.get_format() == OBFormat.MJPG:
                data = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
                color_image = cv2.imdecode(data, cv2.IMREAD_COLOR)

            if color_image is None: return None, None, None, None

            # 2. 处理原始深度图 (uint16)
            raw_depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            raw_depth_data = raw_depth_data.reshape((depth_frame.get_height(), depth_frame.get_width()))
            
            scale = depth_frame.get_depth_scale()
            if scale != 1.0:
                raw_depth_data = (raw_depth_data * scale).astype(np.uint16)

            # 3. 生成深度可视化图 (彩色热力图)
            depth_viz_image = cv2.normalize(raw_depth_data.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX)
            depth_viz_image = cv2.applyColorMap(depth_viz_image.astype(np.uint8), cv2.COLORMAP_JET)
            
            # 确保尺寸一致 (对齐模式下通常是一致的，防万一)
            if color_image.shape[:2] != depth_viz_image.shape[:2]:
                depth_viz_image = cv2.resize(depth_viz_image, (color_image.shape[1], color_image.shape[0]))
                
            # 4. 生成叠加预览图
            overlay_image = cv2.addWeighted(color_image, 0.5, depth_viz_image, 0.5, 0)
            
            # 返回 4 个数据
            return color_image, raw_depth_data, depth_viz_image, overlay_image
            
        except Exception as e:
            return None, None, None, None

    def save_frame(self, color_image, raw_depth_data, depth_viz_image, save_dir, prefix="capture"):
        
        color_dir = os.path.join(save_dir, "color")
        depth_dir = os.path.join(save_dir, "depth")
        viz_dir = os.path.join(save_dir, "viz")

        os.makedirs(color_dir, exist_ok=True)      
        os.makedirs(depth_dir, exist_ok=True)
        os.makedirs(viz_dir, exist_ok=True)
        os.makedirs(save_dir, exist_ok=True)
        timestamp = int(time.time())
        
        color_path = os.path.join(color_dir, f"{prefix}_{timestamp}_color.png")
        depth_path = os.path.join(depth_dir, f"{prefix}_{timestamp}_depth_raw.png")
        viz_path = os.path.join(viz_dir, f"{prefix}_{timestamp}_depth_viz.png")
        
        cv2.imwrite(color_path, color_image)
        cv2.imwrite(depth_path, raw_depth_data)   # 保存16位原始深度(黑图)
        cv2.imwrite(viz_path, depth_viz_image)    # 保存可视化热力图
        
        print(f"已保存三张图像到 {save_dir}, 时间戳: {timestamp}")

    def run_realtime_viewer(self):
        """
        开启实时预览窗口 (4个窗口)
        """
        print("---------------------------------------------")
        print("  Gemini335 Realtime Viewer (4 Windows)")
        print("  [ESC] / [q]: 退出")
        print("  [s]: 保存当前帧 (3张图)")
        print("---------------------------------------------")
        
        # 定义四个窗口名称
        win_color = "1. Color (RGB)"
        win_depth_raw = "2. Raw Depth (uint16 mm)"
        win_depth_viz = "3. Depth Viz (Heatmap)"
        win_overlay = "4. Overlay (Align Check)"
        
        # 创建窗口并允许调整大小
        cv2.namedWindow(win_color, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_depth_raw, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_depth_viz, cv2.WINDOW_NORMAL)
        cv2.namedWindow(win_overlay, cv2.WINDOW_NORMAL)
        
        try:
            while True:
                # 获取4个数据
                color, raw_depth, depth_viz, overlay = self.capture_frame()
                
                if color is not None:
                    # 1. 显示原图
                    cv2.imshow(win_color, color)
                    
                    # 2. 显示原始深度图 (特殊处理)
                    # 因为是uint16，直接显是黑的。为了显示，临时转成8位灰度以便肉眼观察。
                    # 注意：这只是为了显示，不影响保存的原始数据精度。
                    disp_raw = cv2.normalize(raw_depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    cv2.imshow(win_depth_raw, disp_raw)
                    
                    # 3. 显示深度可视化图 (热力图)
                    cv2.imshow(win_depth_viz, depth_viz)
                    
                    # 4. 显示叠加图
                    cv2.imshow(win_overlay, overlay)
                    
                    # 检查按键
                    key = cv2.waitKey(1)
                    if key == 27 or key == ord('q'):
                        print("退出程序...")
                        break
                    elif key == ord('s'):
                        save_dir = "./apple_data"
                        # 保存时传入 3 个必要数据
                        self.save_frame(color, raw_depth, depth_viz, save_dir)
                        
        except KeyboardInterrupt:
            print("用户中断")
        finally:
            cv2.destroyAllWindows()
            self.stop()

    def stop(self):
        try:
            self.pipeline.stop()
        except:
            pass

if __name__ == "__main__":
    try:
        # 初始化相机
        camera = Gemini335(width=1280, height=720)
        # 运行4窗口实时预览
        camera.run_realtime_viewer()
    except Exception as e:
        print(f"程序运行出错: {e}")
        import traceback
        traceback.print_exc()