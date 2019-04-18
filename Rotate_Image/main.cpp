#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

void rotate_arbitrarily_angle(Mat &src, Mat &dst, float angle)
{
	float radian = (float)(angle / 180.0 * CV_PI);

	//���ͼ��
	int maxBorder = (int)(max(src.cols, src.rows)* 1.414); //��Ϊsqrt(2)*max
	int dx = (maxBorder - src.cols) / 2;
	int dy = (maxBorder - src.rows) / 2;
	copyMakeBorder(src, dst, dy, dy, dx, dx, BORDER_CONSTANT);

	//��ת
	Point2f center((float)(dst.cols / 2), (float)(dst.rows / 2));
	Mat affine_matrix = getRotationMatrix2D(center, angle, 1);//�����ת����
	cout << "affine_matrix:\n" << affine_matrix << endl;
	warpAffine(dst, dst, affine_matrix, dst.size());

	//����ͼ����ת֮�����ͼ������ľ���
	float sinVal = abs(sin(radian));
	float cosVal = abs(cos(radian));
	Size targetSize((int)(src.cols * cosVal + src.rows * sinVal),
		(int)(src.cols * sinVal + src.rows * cosVal));

	//��������߿�
	int x = (dst.cols - targetSize.width) / 2;
	int y = (dst.rows - targetSize.height) / 2;
	Rect rect(x, y, targetSize.width, targetSize.height);
	dst = Mat(dst, rect);
}

int main()
{
	Mat image = imread("LM.jpg");
	Mat dst;
	rotate_arbitrarily_angle(image, dst, 90);

	imshow("src", image);
	imshow("dst", dst);
	waitKey(0);

	return 0;

}
