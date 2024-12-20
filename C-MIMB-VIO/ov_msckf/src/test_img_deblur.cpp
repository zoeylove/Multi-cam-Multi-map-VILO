#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // 读取图像
    Mat img_color = imread("/home/tony/Desktop/test/blur1.png");
    if(img_color.empty())
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    // 将彩色图像转换为灰度图像
    Mat img;
    cvtColor(img_color, img, COLOR_BGR2GRAY);

    // 创建图像的傅里叶变换
    Mat blurred_img_dft, blurKernel_dft;
    Mat padded_img, padded_kernel;
    
    // 傅里叶变换前需要扩展图像以避免边缘效应
    int m = getOptimalDFTSize( img.rows );
    int n = getOptimalDFTSize( img.cols ); 
    copyMakeBorder(img, padded_img, 0, m - img.rows, 0, n - img.cols, BORDER_CONSTANT, Scalar::all(0));
    
    // 转换到频域
    dft(padded_img, padded_img, DFT_COMPLEX_OUTPUT);
    
    // 简单的假设：模糊核是一个点扩散函数，这里简化为高斯核（实际应用中可能需要根据实际情况估计）
    Mat kernel = Mat::ones(img.size(), CV_32F) / (float)(img.rows * img.cols);
    dft(kernel, blurKernel_dft, DFT_COMPLEX_OUTPUT);

    // 计算逆卷积核（理论上是卷积核的复共轭除以其频谱的平方模，这里简化处理）
    Mat inverseFilter = blurKernel_dft.clone();
    divide(Scalar::all(1), blurKernel_dft, inverseFilter); // 这里简化处理，实际应考虑能量补偿等问题

    // 应用逆滤波
    mulSpectrums(padded_img, inverseFilter, blurred_img_dft, 0, true);

    // 转回空间域
    Mat result;
    idft(blurred_img_dft, result, DFT_SCALE | DFT_REAL_OUTPUT);

    // 裁剪并显示结果
    result = result(Rect(0, 0, img.cols, img.rows));
    imshow("Original Image", img);
    imshow("Deblurred Image", result);

    waitKey();
    destroyAllWindows();

    return 0;
}