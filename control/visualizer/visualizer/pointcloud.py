from flask import Flask, render_template_string
import numpy as np
import plotly.graph_objs as go
import plotly.io as pio
import matplotlib.cm as cm
from termcolor import cprint
import os

task_cfg_dict = {
            "adroit_hammer_pointcloud": {
                "x_min": -1,
                "x_max": 1,
                "y_min": -1,
                "y_max": 1,
                "z_min": -1,
                "z_max": 1,
                "camera_view": dict(
                    eye=dict(x=-2.2, y=-1.7, z=0.9),
                    center=dict(x=-0.1, y=0.0, z=0.2),
                    up=dict(x=0, y=0, z=1)),
                "size": 4,
            },
            "test": {
                "x_min": -0.5,
                "x_max": 0.5,
                "y_min": 0,
                "y_max": 1,
                "z_min": -1,
                "z_max": 0,
                "camera_view": dict(
                    eye=dict(x=0, y=0.5, z=2),
                    center=dict(x=-0.1, y=0.0, z=0.2),
                    up=dict(x=0, y=0, z=1)),
                "size": 4,
            },
            "hammer": {
                "x_min": -0.5,
                "x_max": 0.5,
                "y_min": -0.5,
                "y_max": 0.5,
                "z_min": -0.2,
                "z_max": 0.5,
                "camera_view": dict(
                    eye=dict(x=0, y=0, z=1.1),
                    center=dict(x=-0.1, y=0.0, z=0.2),
                    up=dict(x=0, y=0, z=1)),
                "size": 4,
            },
        }

def visualize_pointcloud(pointcloud, color:tuple=None):
    vis = Visualizer()
    vis.visualize_pointcloud(pointcloud, color=color)

class Visualizer:
    def __init__(self):
        self.app = Flask(__name__)
        self.pointclouds = []

    def _generate_trace(self, pointcloud, color:tuple=None, size=5, opacity=0.7):
        x_coords = pointcloud[:, 0]
        y_coords = pointcloud[:, 1]
        z_coords = pointcloud[:, 2]

        if pointcloud.shape[1] == 3:
            if color is None:
                # design a colorful point cloud based on 3d coordinates
                # Normalize coordinates to range [0, 1]
                min_coords = pointcloud.min(axis=0)
                max_coords = pointcloud.max(axis=0)
                normalized_coords = (pointcloud - min_coords) / (max_coords - min_coords)
                try:
                    # Use normalized coordinates as RGB values
                    colors = ['rgb({},{},{})'.format(int(r*255), int(g*255), int(b*255)) for r, g, b in normalized_coords]
                except: # maybe meet NaN error
                    # use simple cyan color
                    colors = ['rgb(0,255,255)' for _ in range(len(x_coords))]
            else:
                colors = ['rgb({},{},{})'.format(color[0], color[1], color[2]) for _ in range(len(x_coords))]
        else:
            colors = ['rgb({},{},{})'.format(int(r), int(g), int(b)) for r, g, b in pointcloud[:, 3:6]]

        return go.Scatter3d(
            x=x_coords,
            y=y_coords,
            z=z_coords,
            mode='markers',
            marker=dict(
                size=size,
                opacity=opacity,
                color=colors
            )
        )


    def colorize(self, pointcloud):
        if pointcloud.shape[1] == 3:

            # design a colorful point cloud based on 3d coordinates
            # Normalize coordinates to range [0, 1]
            min_coords = pointcloud.min(axis=0)
            max_coords = pointcloud.max(axis=0)
            normalized_coords = (pointcloud - min_coords) / (max_coords - min_coords)
            try:
                # Use normalized coordinates as RGB values
                colors = ['rgb({},{},{})'.format(int(r*255), int(g*255), int(b*255)) for r, g, b in normalized_coords]
            except: # maybe meet NaN error
                # use simple cyan color
                x_coords = pointcloud[:, 0]
                colors = ['rgb(0,255,255)' for _ in range(len(x_coords))]

        else:
            colors = ['rgb({},{},{})'.format(int(r), int(g), int(b)) for r, g, b in pointcloud[:, 3:6]]
        return colors


    def visualize_pointcloud(self, pointcloud, color:tuple=None):
        trace = self._generate_trace(pointcloud, color=color, size=6, opacity=1.0)
        layout = go.Layout(margin=dict(l=0, r=0, b=0, t=0))
        fig = go.Figure(data=[trace], layout=layout)

        fig.update_layout(

            scene=dict(
                # aspectmode='cube',
                xaxis=dict(
                    showbackground=False,  # 隐藏背景网格
                    showgrid=True,        # 隐藏网格
                    showline=True,         # 显示轴线
                    linecolor='grey',      # 设置轴线颜色为灰色
                    zerolinecolor='grey',  # 设置0线颜色为灰色
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色

                ),
                yaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色
                ),
                zaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色
                ),
                bgcolor='white'  # 设置背景色为白色
            )
        )
        div = pio.to_html(fig, full_html=False)

        @self.app.route('/')
        def index():
            return render_template_string('''<div>{{ div|safe }}</div>''', div=div)

        self.app.run(debug=True, use_reloader=False)


    def visualize_sequence(self, pointclouds, task_name=None, save_dir=None):

        os.makedirs(save_dir, exist_ok=True)

        axis_visible = False

        """
        the axis range and the camera view should be set manually for each task
        """

        # axis range
        x_min, x_max = task_cfg_dict[task_name]["x_min"], task_cfg_dict[task_name]["x_max"]
        y_min, y_max = task_cfg_dict[task_name]["y_min"], task_cfg_dict[task_name]["y_max"]
        z_min, z_max = task_cfg_dict[task_name]["z_min"], task_cfg_dict[task_name]["z_max"]
        # camera view
        camera_view = task_cfg_dict[task_name]["camera_view"]
        size = task_cfg_dict[task_name]["size"]
        opacity = 1.0

        # visualize pointclouds as videos and save
        for step_idx, pointcloud in enumerate(pointclouds):

            fig = go.Figure()
            x = pointcloud[:, 0]
            y = pointcloud[:, 1]
            z = pointcloud[:, 2]

            colors = self.colorize(pointcloud)

            fig.add_trace(go.Scatter3d(
                x=x, y=y, z=z,
                mode='markers',
                marker=dict(
                    color=colors,
                    size=size,
                    opacity=opacity,
                )
            ))

            # Update layout for this specific figure
            fig.update_layout(
                    scene=dict(
                        camera=camera_view,
                        xaxis=dict(
                            showbackground=axis_visible,
                            showticklabels=axis_visible,  # Hide axis tick labels
                            visible=axis_visible,         # Hide the axis line and ticks
                            range=[x_min, x_max]
                        ),
                        yaxis=dict(
                            showbackground=axis_visible,
                            showticklabels=axis_visible,
                            visible=axis_visible,
                            range=[y_min, y_max]
                        ),
                        zaxis=dict(
                            showbackground=axis_visible,
                            showticklabels=axis_visible,
                            visible=axis_visible,
                            range=[z_min, z_max]
                        ),
                        bgcolor='white'
                    )
                )

            # save the html
            fig.write_html(f"{save_dir}/frame_{step_idx:03d}.html")
            # Save the figure
            fig.write_image(f"{save_dir}/frame_{step_idx:03d}.png")

        os.system(f"ffmpeg -r 10 -f image2 -s 1920x1080 -i {save_dir}/frame_%03d.png -vcodec libx264 -pix_fmt yuv420p {save_dir}/trajectory_video.mp4")

        # os.system(f"ffmpeg -r 10 -f image2 -s 1920x1080 -i {save_dir}/frame_%03d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p {save_dir}/trajectory_video.mp4")
        cprint(f"save video to {save_dir}/trajectory_video.mp4", 'green')


    def save_visualization_to_file(self, pointcloud, file_path, color:tuple=None):
        # visualize pointcloud and save as html
        trace = self._generate_trace(pointcloud, color=color)
        layout = go.Layout(margin=dict(l=0, r=0, b=0, t=0))
        fig_html = pio.to_html(go.Figure(data=[trace], layout=layout), full_html=True)

        with open(file_path, 'w') as file:
            file.write(fig_html)
        print(f"Visualization saved to {file_path}")

    def save_train_test_visualization_to_file(self, train_points, test_points, title, file_path):
        # visualize for generalization

        # 创建训练点的轨迹
        # 27a2c3
        blue_color = (39, 162, 195)

        # 99e0eb
        # blue_color = (153, 224, 235)

        # fe7b5b
        red_color = (254, 123, 91)

        # tdmpc red: D5262C
        # red_color = (213, 38, 44)
        # tdmpc blue: 1A78B1
        # blue_color = (26, 120, 177)

        # shape: ['circle', 'circle-open', 'cross', 'diamond',
        #     'diamond-open', 'square', 'square-open', 'x']

        point_size = 5
        train_trace = self._generate_trace(train_points, color=red_color,
                                           size=point_size, opacity=1.0)
        train_trace['marker']['symbol'] = 'circle'
        train_trace['name'] = 'Train'  # 图例标签

        # 创建测试点的轨迹
        if test_points.shape != (0,):
            test_trace = self._generate_trace(test_points, color=blue_color,
                                              size=point_size, opacity=0.7)
            test_trace['marker']['symbol'] = 'circle'
            test_trace['name'] = 'Test'  # 图例标签
        else:
            print("test_points.shape == (0,).")

            test_trace = go.Scatter3d(
                x=[],
                y=[],
                z=[],
                mode='markers',
                marker=dict(
                    size=point_size,
                    opacity=0.8,
                    color='rgb(0,0,0)'
                ),
                name='Test'
            )

        # 创建图形并设置布局
        layout = go.Layout(
                    margin=dict(l=0, r=0, b=0, t=0),
                    showlegend=True,
                    legend=dict(
                        x=0.8,
                        y=0.74,
                        xanchor='center',
                        yanchor='top',
                        orientation='v',
                        bordercolor='Black',
                        font=dict(size=13),
                    ),
                    annotations=[
                        dict(
                            x=0.5,
                            y=0.76,  # 调整此值以将标题放在图例上方
                            xanchor='center',
                            yanchor='bottom',
                            text=title,  # 这里是您的图例标题
                            showarrow=False,
                            font=dict(size=20)
                        )
                    ]
                )


        fig = go.Figure(data=[train_trace, test_trace], layout=layout)

        x_min, x_max = -0.1, 0.1
        y_min, y_max = 0.7, 0.9
        z_min, z_max = 0.1, 0.3

        fig.update_layout(
            scene=dict(
                camera=dict(
                    eye=dict(x=-2.2, y=-1.7, z=0.9),  # 调整这些值来设置初始视角
                    center=dict(x=-0.1, y=0.0, z=0.2),        # 视图的中心点
                    up=dict(x=0, y=0, z=1)             # 定义“上”方向
                ),
                aspectmode='cube',
                xaxis=dict(
                    showbackground=False,  # 隐藏背景网格
                    showgrid=True,        # 隐藏网格
                    showline=True,         # 显示轴线
                    linecolor='grey',      # 设置轴线颜色为灰色
                    zerolinecolor='grey',  # 设置0线颜色为灰色
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色
                    range=[x_min, x_max],
                    # set fontsize of 刻度
                    tickfont=dict(
                        size=11,
                    ),

                ),
                yaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色
                    range=[y_min, y_max],
                    tickfont=dict(
                        size=11,
                    ),
                ),
                zaxis=dict(
                    showbackground=False,
                    showgrid=True,
                    showline=True,
                    linecolor='grey',
                    zerolinecolor='grey',
                    zeroline=False,        # 关闭0线
                    gridcolor='grey',      # 设置网格颜色为灰色
                    range=[z_min, z_max],
                    tickfont=dict(
                        size=11,
                    ),
                ),
                bgcolor='white'  # 设置背景色为白色
            )
        )


        # 将图形保存为HTML文件
        fig_html = pio.to_html(fig, full_html=True)

        # 保存为png
        # high resolution
        fig.write_image(file_path.replace('html', 'pdf'))

        with open(file_path, 'w') as file:
            file.write(fig_html)
        print(f"Visualization saved to {file_path}")
