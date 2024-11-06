import pandas as pd
import numpy as np

# 读取数据
data = pd.read_csv('ladar_data.csv', header=None, names=['angle', 'distance'])

# 确保角度在0到2*pi之间
data['angle'] = data['angle'] % (2 * np.pi)

# 在计算最小值之前，去掉为0.0的值
data_filtered = data[data['distance'] != 0.0]

# 按角度分组，计算均值、方差、计数、最大值和最小值（去掉0.0的值）
result = data_filtered.groupby('angle')['distance'].agg(['mean', 'var', 'count', 'max', 'min']).reset_index()

# 计算 (max - min) / 2
result['precision'] = (result['max'] - result['min']) / 2

# 重命名列
result.columns = ['angle', 'mean_distance', 'variance_distance', 'count_distance', 'max_distance', 'min_distance', 'precision']

# 输出结果
print(result)

# 如果需要保存结果到CSV文件
result.to_csv('ladar_data_analysis.csv', index=False)
