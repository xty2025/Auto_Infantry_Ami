import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def read_custom_data(filename):
    """读取特定格式的数据文件"""
    centerx, centery, imu_yaw, imu_pitch = [], [], [], []
    
    with open(filename, 'r') as f:
        for line in f:
            # 分割每行数据
            items = line.strip().split(',')
            if len(items) != 4 :
                continue
            ##每次pitch和yawl读到4个数据
            ##add
            # if len(items) != 4 or ':' not in items[0] or ':' not in items[1] or ':' not in items[2] or ':' not in items[3]:
            #     continue
            # num_str = [item.split(':')[1] for item in items]
            #items = [float(num) for num in num_str]
            items = [float(num) for num in items]
            centerx.append(items[0])
            centery.append(items[1])
            imu_yaw.append(items[2])
            imu_pitch.append(items[3])
    
    return np.array(centerx), np.array(centery), np.array(imu_yaw), np.array(imu_pitch)


def linear_func(x, A, B, C):
    """线性函数模型"""
    return A * x[0] + B * x[1] + C

def square_func(x, A, B, C, D):
    """二次函数模型"""
    return A * x[0] * x[1] + B * x[0] + C * x[1] + D 

def calculate_residuals(y_true, y_pred):
    """计算残差"""
    return np.average(np.abs(y_true - y_pred))
  
    #标准的
def calculate_std_residuals(y_true,y_pred):
    #return np.std(y_true - y_pred)
    return sqrt(np.sum(y_true-y_pred)**2/len(y_true))   


def calculate_residuals_mean(y_true, y_pred):
    y_pred_mod = y_pred + np.mean(y_true) - np.mean(y_pred)
    return np.average(np.abs(y_true - y_pred_mod))


def main(train_filename,test=False,test_filename=None):
    # 读取数据
    x, x1, _, y = read_custom_data(train_filename)
    print("data size:", len(x), len(x1), len(y))
    # 拟合线性模型
    popt_linear, _ = curve_fit(linear_func, (x, x1), y)
    X1, Y1, _ = popt_linear

    # 拟合二次模型
    popt_square, _ = curve_fit(square_func, (x, x1), y)

    # 计算残差
    y_pred_linear = linear_func((x, x1), *popt_linear)
    y_pred_square = square_func((x, x1), *popt_square)
    residuals_linear = calculate_residuals(y, y_pred_linear)
    residuals_square = calculate_residuals(y, y_pred_square)

    print('线性模型拟合参数:', popt_linear)
    print('二次模型拟合参数:', popt_square)
    print('线性模型残差:', residuals_linear)
    print('二次模型残差:', residuals_square)

    if test:
        # 读取测试数据
        x_test, x1_test, _, y_test = read_custom_data(test_filename)
        y_pred_linear_test = linear_func((x_test, x1_test), *popt_linear)
        y_pred_square_test = square_func((x_test, x1_test), *popt_square)
        residuals_linear_test = calculate_residuals(y_test, y_pred_linear_test)
        residuals_square_test = calculate_residuals(y_test, y_pred_square_test)
        residuals_linear_test_mean = calculate_residuals_mean(y_test, y_pred_linear_test)
        residuals_square_test_mean = calculate_residuals_mean(y_test, y_pred_square_test)

        print('线性模型测试残差:', residuals_linear_test)
        print('二次模型测试残差:', residuals_square_test)
        print('线性模型测试残差均值:', residuals_linear_test_mean)
        print('二次模型测试残差均值:', residuals_square_test_mean)
        # 读取数据
    x, x1, y, _ = read_custom_data('1.txt')
    ##把读到的数据login-info到1.txt中。
    print("data size:", len(x), len(x1), len(y))
    # 拟合线性模型
    popt_linear, _ = curve_fit(linear_func, (x, x1), y)
    X2, Y2, _ = popt_linear

    # 拟合二次模型
    popt_square, _ = curve_fit(square_func, (x, x1), y)

    # 计算残差
    y_pred_linear = linear_func((x, x1), *popt_linear)
    y_pred_square = square_func((x, x1), *popt_square)
    residuals_linear = calculate_residuals(y, y_pred_linear)
    residuals_square = calculate_residuals(y, y_pred_square)

    print('线性模型拟合参数:', popt_linear)
    print('二次模型拟合参数:', popt_square)
    print('线性模型残差:', residuals_linear)
    print('二次模型残差:', residuals_square)

    if test:
        # 读取测试数据
        x_test, x1_test, y_test, _ = read_custom_data(test_filename)
        y_pred_linear_test = linear_func((x_test, x1_test), *popt_linear)
        y_pred_square_test = square_func((x_test, x1_test), *popt_square)
        residuals_linear_test = calculate_residuals(y_test, y_pred_linear_test)
        residuals_square_test = calculate_residuals(y_test, y_pred_square_test)
        residuals_linear_test_mean = calculate_residuals_mean(y_test, y_pred_linear_test)
        residuals_square_test_mean = calculate_residuals_mean(y_test, y_pred_square_test)

        print('线性模型测试残差:', residuals_linear_test)
        print('二次模型测试残差:', residuals_square_test)
        print('线性模型测试残差均值:', residuals_linear_test_mean)
        print('二次模型测试残差均值:', residuals_square_test_mean)
    return X1, Y1, X2, Y2

if __name__ == '__main__':
    X1, Y1, X2, Y2 = main('1.txt', test=True, test_filename='2.txt')
    print(X1, Y1, X2, Y2)