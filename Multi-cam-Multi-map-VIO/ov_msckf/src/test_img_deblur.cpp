#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    Mat img_color = imread("/home/tony/Desktop/test/blur1.png");
    if(img_color.empty())
    {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    Mat img;
    cvtColor(img_color, img, COLOR_BGR2GRAY);

    Mat blurred_img_dft, blurKernel_dft;
    Mat padded_img, padded_kernel;
    
    int m = getOptimalDFTSize( img.rows );
    int n = getOptimalDFTSize( img.cols ); 
    copyMakeBorder(img, padded_img, 0, m - img.rows, 0, n - img.cols, BORDER_CONSTANT, Scalar::all(0));
    
    dft(padded_img, padded_img, DFT_COMPLEX_OUTPUT);
    
    Mat kernel = Mat::ones(img.size(), CV_32F) / (float)(img.rows * img.cols);
    dft(kernel, blurKernel_dft, DFT_COMPLEX_OUTPUT);

    Mat inverseFilter = blurKernel_dft.clone();
    divide(Scalar::all(1), blurKernel_dft, inverseFilter);

    mulSpectrums(padded_img, inverseFilter, blurred_img_dft, 0, true);

    Mat result;
    idft(blurred_img_dft, result, DFT_SCALE | DFT_REAL_OUTPUT);

    result = result(Rect(0, 0, img.cols, img.rows));
    imshow("Original Image", img);
    imshow("Deblurred Image", result);

    waitKey();
    destroyAllWindows();

    return 0;
}