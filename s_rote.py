import math
import yaml
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union

from farmland_path_planning import Coordinateself


# === 参数设置 ===
working_wide = 6            # 作业宽度
interpolation_step = 0.05    # 插值步长
yaml_file = './a.yaml'  # 当前文件夹下的路径地址
# 原始田块边界点
# or_points = [(-40.0, -3.1), (0.0, -3.0), (5.1, 27.0), (-35.0, 27.0)]
# or_points = [(50.7, 10.3), (170.1, 1.), (150.2, 70.0), (10.7, 80.46)]
# or_points = [(50.7, 20.3), (120.1, 1.2), (150.2, 46.0), (10.7, 80.46)]
or_points = [(50.7, 5.3), (120.1, 21.2), (150.2, 46.0), (10.7, 80.46)]
# or_points = [(12.7, 10.3), (150.1, 1.), (158.2, 100.0), (30.7, 68.46)]
# or_points = [(0.0,0.1), (10.0,0.0),(10.1,10.0),(0.0,10.0)]


# === 路径生成模块 ===
transformer = Coordinateself()
ass, angel_for_back = transformer.s_rote(or_points, working_wide)# 旋转田块，生成基本航迹点
# 优化掉头点
last_p = ass[-1]
ok_l = ass[:-1] if len(ass) % 2 != 0 else ass
# 提取路径点并添加起始与终止点
path_li = transformer.point_extraction(ok_l, working_wide)
path_li.append(last_p)
path_li.insert(0, ass[0])
# 坐标反变换
dx, dy = float(or_points[0][0]), float(or_points[0][1])
path_list = transformer.back_transform(path_li, -angel_for_back)
path_list = [(ax + dx, ay + dy) for ax, ay in path_list]



# 保存路径到 YAML 文件中（供其他模块使用）
yaml_path_list = [list(point) for point in path_list]
with open(yaml_file, 'w') as file:
    formatted_path_str = "[\n" + ",\n".join("  " + str(item) for item in yaml_path_list) + "\n]"
    file.write(formatted_path_str)



# === 覆盖率计算模块 ===
def interpolate_between_points(p1, p2, step=interpolation_step):
    x1, y1 = p1
    x2, y2 = p2
    distance = math.hypot(x2 - x1, y2 - y1)
    if distance < step:
        return [p1, p2]
    num_points = int(math.ceil(distance / step)) + 1
    return [(x1 + i * (x2 - x1) / (num_points - 1),
             y1 + i * (y2 - y1) / (num_points - 1)) for i in range(num_points)]

def interpolate_path(path, step=interpolation_step):
    interpolated_path = []
    for i in range(len(path) - 1):
        interpolated_path.extend(interpolate_between_points(path[i], path[i + 1], step)[:-1])
    interpolated_path.append(path[-1])
    return interpolated_path

def path_to_polygon(path, width):
    polygons = []
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        ux, uy = -dy / length, dx / length
        p1 = (x1 + ux * width / 2, y1 + uy * width / 2)
        p2 = (x1 - ux * width / 2, y1 - uy * width / 2)
        p3 = (x2 - ux * width / 2, y2 - uy * width / 2)
        p4 = (x2 + ux * width / 2, y2 + uy * width / 2)
        polygons.append(Polygon([p1, p2, p3, p4]))
    # 合并并确保路径为有效几何体
    union_result = unary_union(polygons)
    if isinstance(union_result, Polygon):
        return union_result
    else:
        return unary_union([poly for poly in union_result if isinstance(poly, Polygon)])

def calculate_coverage(field_vertices, path, width):
    field_polygon = Polygon(field_vertices)
    interpolated_path = interpolate_path(path)
    work_area = path_to_polygon(interpolated_path, width)
    covered_area = work_area.intersection(field_polygon)
    coverage = (covered_area.area / field_polygon.area) * 100
    return coverage, covered_area

def visualize(field_vertices, path, covered_area, coverage):
    fig, ax = plt.subplots(figsize=(8, 8))
    field_polygon = Polygon(field_vertices)
    fx, fy = field_polygon.exterior.xy
    ax.fill(fx, fy, color='green', alpha=0.5, label='Field Area')

    path_line = LineString(path)
    x, y = path_line.xy
    ax.plot(x, y, color='black', linewidth=2, label='Path')

    interp_line = LineString(interpolate_path(path))
    x, y = interp_line.xy
    ax.plot(x, y, color='red', linestyle='--', linewidth=1)

    if not covered_area.is_empty:
        try:
            x, y = covered_area.exterior.xy
            ax.fill(x, y, color='black', alpha=0.2, label='Covered Area')
        except:
            for geom in covered_area.geoms:
                x, y = geom.exterior.xy
                ax.fill(x, y, color='black', alpha=0.2)

    ax.set_title(f'Field Coverage Rate: {coverage:.2f}%', fontsize=16)
    ax.set_xlabel('X/m', fontsize=16)
    ax.set_ylabel('Y/m', fontsize=16)
    ax.legend(fontsize=12)
    ax.grid(True)
    ax.set_aspect('equal')
    plt.show()

# === 主流程 ===
def main():
    path = path_list  # 直接使用内存中的路径数据
    coverage, covered_area = calculate_coverage(or_points, path, working_wide)
    print(f"覆盖率: {coverage:.2f}%")
    visualize(or_points, path, covered_area, coverage)

if __name__ == '__main__':
    main()
