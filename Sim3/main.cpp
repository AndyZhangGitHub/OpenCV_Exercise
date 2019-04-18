/****************************************************************
*   文件名： Sim3.cpp
*   文件作者：Andy
*   联系方式：675427451@qq.com
*   创建时间：2019/04/18
*   描述说明：
*       根据三维空间中的匹配点集计算相似变换矩阵。ORB_SLAM2中在闭环检
*       测阶段用到此算法。
*
*****************************************************************/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//************************************     
// 函数名称: ComputeSim3     
// 函数说明：计算相似变换矩阵     
// 作 成 者：Mr.Andy     
// 作成日期：2019/04/18     
// 返 回 值: Mat     
// 参    数: Mat &  , Mat &     
//************************************  /
Mat ComputeSim3(Mat &P1, Mat &P2)
{
	Mat Pr1(P1.size(), CV_32F);
	Mat Pr2(P2.size(), CV_32F);
	Mat O1(3, 1, CV_32F);
	Mat O2(3, 1, CV_32F);

	cv::reduce(P1, O1, 1, CV_REDUCE_AVG);// 计算质心
	cout << "O1: \n" << O1 << endl;
	for (int i = 0; i < P1.cols; i++)
	{
		Pr1.col(i) = P1.col(i) - O1;// 去质心
	}
	cout << "Pr1:\n " << Pr1 << endl;

	cv::reduce(P2, O2, 1, CV_REDUCE_AVG);
	for (int i = 0; i < P2.cols; i++)
	{
		Pr2.col(i) = P2.col(i) - O2;
	}


	Mat M = Pr2*Pr1.t();// 计算M矩阵
	cout << "M:\n " << M << endl;

	// 计算N矩阵
	double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;
	Mat N(4, 4, CV_32F);

	N11 = M.at<float>(0, 0) + M.at<float>(1, 1) + M.at<float>(2, 2);
	N12 = M.at<float>(1, 2) - M.at<float>(2, 1);
	N13 = M.at<float>(2, 0) - M.at<float>(0, 2);
	N14 = M.at<float>(0, 1) - M.at<float>(1, 0);
	N22 = M.at<float>(0, 0) - M.at<float>(1, 1) - M.at<float>(2, 2);
	N23 = M.at<float>(0, 1) + M.at<float>(1, 0);
	N24 = M.at<float>(2, 0) + M.at<float>(0, 2);
	N33 = -M.at<float>(0, 0) + M.at<float>(1, 1) - M.at<float>(2, 2);
	N34 = M.at<float>(1, 2) + M.at<float>(2, 1);
	N44 = -M.at<float>(0, 0) - M.at<float>(1, 1) + M.at<float>(2, 2);

	N = (Mat_<float>(4, 4) << N11, N12, N13, N14,
		N12, N22, N23, N24,
		N13, N23, N33, N34,
		N14, N24, N34, N44);

	// N矩阵特征分解
	Mat eval, evec;
	cv::eigen(N, eval, evec);

	// N矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3）
	// 将(q1 q2 q3)放入vec行向量，vec就是四元数旋转轴乘以sin(ang/2)
	Mat vec(1, 3, CV_32F);
	(evec.row(0).colRange(1, 4)).copyTo(vec);
	double ang = 2 * atan2(norm(vec), evec.at<float>(0, 0));
	vec = ang * vec / norm(vec);

	Mat R12(3, 3, P1.type());
	cv::Rodrigues(vec, R12);

	// 计算尺度
	Mat P3 = R12*Pr2;
	float nom = Pr1.dot(P3);

	Mat aux_P3(P3.size(), CV_32F);
	aux_P3 = P3;
	cv::pow(P3, 2, aux_P3);

	float den = 0;
	for (int i = 0; i < aux_P3.rows; i++)
	{
		for (int j = 0; j < aux_P3.cols; j++)
		{
			den += aux_P3.at<float>(i, j);
		}
	}

	float s12 = nom / den;

	// 计算平移
	Mat t12(1, 3, CV_32F);
	t12 = O1 - s12*R12*O2;

	// 构造相似变换矩阵
	Mat T12 = Mat::eye(4, 4, CV_32F);
	Mat sR = s12*R12;
	sR.copyTo(T12.rowRange(0, 3).colRange(0, 3));
	t12.copyTo(T12.rowRange(0, 3).col(3));

	Mat T21 = Mat::eye(4, 4, CV_32F);
	Mat sR_inv = (1.0 / s12)*R12.t();
	sR_inv.copyTo(T21.rowRange(0, 3).colRange(0, 3));
	Mat t_inv = -sR_inv*t12;
	t_inv.copyTo(T21.rowRange(0, 3).col(3));

	// 验证
	cv::Rodrigues(R12.inv(), vec);
	cout << "s21: " << 1 / s12 << endl
		<< "R21: " << (vec * 180 / CV_PI).t() << endl
		<< "t21: " << t_inv.t() << endl;

	return T12.clone();

}

// Sim3仿真
void main()
{

	Mat p1_c1 = (Mat_<float>(3, 1) << 0, 0, 20);
	Mat p2_c1 = (Mat_<float>(3, 1) << 2, 4, 30);
	Mat p3_c1 = (Mat_<float>(3, 1) << 5, 9, 40);

	float s21 = 0.7;
	Mat rotation_vector = (Mat_<float>(3, 1) << 0, 2 * CV_PI / 180, 0);//绕y轴旋转2度
	Mat R21(3, 3, CV_32F);
	Rodrigues(rotation_vector, R21);//将旋转向量转为旋转矩阵
	Mat t21 = (Mat_<float>(3, 1) << 0.5, 0.3, 0.1);//平移向量

	Mat p1_c2 = s21*R21*p1_c1 + t21;
	Mat p2_c2 = s21*R21*p2_c1 + t21;
	Mat p3_c2 = s21*R21*p3_c1 + t21;

	Mat P_c1(3, 3, CV_32F);
	Mat P_c2(3, 3, CV_32F);

	p1_c1.copyTo(P_c1.col(0));
	p2_c1.copyTo(P_c1.col(1));
	p3_c1.copyTo(P_c1.col(2));

	p1_c2.copyTo(P_c2.col(0));
	p2_c2.copyTo(P_c2.col(1));
	p3_c2.copyTo(P_c2.col(2));


	cout << "P_c1:\n" << P_c1 << endl;
	cout << "P_c2:\n" << P_c2 << endl;

	ComputeSim3(P_c1, P_c2);

	getchar();
}



