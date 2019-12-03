/*!
 * @file Kinematics.cpp
 * @brief 運動学を計算するプログラム
 * @date 2014.1.1
 * @author Yasuo Hayashibara
 */

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "Constant.h"
#include "Kinematics.h"

using namespace GankenKun2018;
using namespace Const;

Matrix3f rodrigues(Vector3f a, float q){
	Matrix3f ret;
	float cos_q = cos(q), sin_q = sin(q);
	ret(0,0) = a(0)*a(0)*(1.0f-cos_q) +      cos_q;
	ret(0,1) = a(0)*a(1)*(1.0f-cos_q) - a(2)*sin_q;
	ret(0,2) = a(2)*a(0)*(1.0f-cos_q) + a(1)*sin_q;
	ret(1,0) = a(0)*a(1)*(1.0f-cos_q) + a(2)*sin_q;
	ret(1,1) = a(1)*a(1)*(1.0f-cos_q) +      cos_q;
	ret(1,2) = a(1)*a(2)*(1.0f-cos_q) - a(0)*sin_q;
	ret(2,0) = a(2)*a(0)*(1.0f-cos_q) - a(1)*sin_q;
	ret(2,1) = a(1)*a(2)*(1.0f-cos_q) + a(0)*sin_q;
	ret(2,2) = a(2)*a(2)*(1.0f-cos_q) +      cos_q;

	return ret;
}

/*!
 * @brief コンストラクタ
 * データを初期化する
 */
Kinematics::Kinematics(class Link *link)
{
	this->link = link;
}

/*!
 * @brief 順運動学を再帰的に計算するための関数
 */
void Kinematics::ForwardKinematics(int n)
{
	if (n == Const::NON) return;
	if (n != Const::CC ) {
		int i = link[n].mother;
		link[n].p = link[i].R * link[n].b + link[i].p;
		link[n].R = link[i].R * rodrigues(link[n].a, link[n].q);
	}
	ForwardKinematics(link[n].sister);
	ForwardKinematics(link[n].child);
}

/*
 * @brief 順運動学の計算
 */
void Kinematics::calcForwardKinematics()
{
	ForwardKinematics(Const::CC);
}

/*
 * @brief 関節角の設定
 *
 * @param[in] 関節の角度列(rad)（サーボモータのIDに対応するデータ）
 */
void Kinematics::setJointAngle(float *angle)
{
	link[Const::LR2].q =  angle[Const::ANKLE_ROLL_L ];	// 左足首ロール軸
	link[Const::LP4].q =  angle[Const::ANKLE_PITCH_L] - angle[Const::SHIN_PITCH_L] ;	// 左足首ピッチ軸
	link[Const::LP3].q =  angle[Const::SHIN_PITCH_L ];  // 左足膝上ピッチ軸（平行リンク）
	link[Const::LP2].q = -angle[Const::THIGH_PITCH_L];	// 左足膝上ピッチ軸（平行リンク）
	link[Const::LP1].q =  angle[Const::THIGH_PITCH_L];  // 左股ピッチ軸（平行リンク）
	link[Const::LR1].q =  angle[Const::HIP_ROLL_L ];	// 左股ロール軸
	link[Const::LY ].q =  angle[Const::HIP_YAW_L  ];	// 左股ヨー軸

	link[Const::RR2].q =  angle[Const::ANKLE_ROLL_R ];	// 右足首ロール軸
	link[Const::RP4].q =  angle[Const::ANKLE_PITCH_R] - angle[Const::SHIN_PITCH_R] ;	// 右足首ピッチ軸
	link[Const::RP3].q =  angle[Const::SHIN_PITCH_R ];  // 右足膝上ピッチ軸（平行リンク）
	link[Const::RP2].q = -angle[Const::THIGH_PITCH_R];	// 右足膝上ピッチ軸（平行リンク）
	link[Const::RP1].q =  angle[Const::THIGH_PITCH_R];  // 右股ピッチ軸（平行リンク）
	link[Const::RR1].q =  angle[Const::HIP_ROLL_R ];	// 右股ロール軸
	link[Const::RY ].q =  angle[Const::HIP_YAW_R  ];	// 右股ヨー軸

	link[Const::LSP].q =  angle[Const::ARM_PITCH_L];	// 左肩ピッチ軸
	link[Const::LSR].q =  angle[Const::ARM_ROLL_L ];	// 左肩ロール軸

	link[Const::RSP].q =  angle[Const::ARM_PITCH_R];	// 右肩ピッチ軸
	link[Const::RSR].q =  angle[Const::ARM_ROLL_R ];	// 右肩ロール軸

	link[Const::SH ].q =  angle[Const::HEAD_YAW   ];	// 首ヨー軸
	link[Const::HP ].q =  angle[Const::HEAD_PITCH ];	// 首ピッチ軸（Acceliteは無い）
}

#if 0

// 逆運動学の定式化を試行錯誤していたときのもの
// 数値解法の処理時間が問題なければ削除

float sign(float val){
	float ret;
	if (val >= 0.0f) ret =  1.0f;
	else             ret = -1.0f;
	return ret;
}

/*
 * @brief 順運動学の計算
 * 足首(RR2,LR2)（足先ではない）の位置・姿勢と腰(CC)の位置・姿勢から角度を計算
 */

// 股の軸がずれているため，正確な数値ではない．

void Kinematics::calcInverseKinematics(float *angle)
{
	float A, B, C, c5, cz, sz;
	float q2, q3, q4, q5, q6, q6a, q7, qp;
	Vector3f r, Rroll(1,0,0), Rpitch(0,1,0);
	Matrix3f R;
	Link *Body  = &link[Const::CC ];
	Link *RFoot = &link[Const::RP4], *LFoot = &link[Const::LR2];
	Link *RHip  = &link[Const::RY ], *LHip  = &link[Const::LY ];

	// 右足の計算
	A = fabs(link[Const::RP2].b(2));		// 股から膝までの距離 (m)
	B = fabs(link[Const::RP4].b(2));		// 膝から足首までの距離 (m)
	
	r = RFoot->R.transpose() * (Body->p + Body->R * RHip->b - RFoot->p);
														// 足首から股関節のベクトル
	std::cout << "r:\n" << r << std::endl << std::endl;
	std::cout << "RHip->b:\n" << RHip->b << std::endl << std::endl;
	C = r.norm() - fabs(link[Const::RP3].b(2));
														// 膝の長さを引いた距離
	std::cout << C << std::endl << A << std::endl << B << std::endl << std::endl;
	c5 = (C*C-A*A-B*B)/(2.0f*A*B);
	std::cout << c5 << std::endl << std::endl;
	if      (c5 >=  1.0f) q5 = 0.0f;
	else if (c5 <= -1.0f) q5 = (float)M_PI;
	else                  q5 = (float)acos(c5);			// 膝のピッチ角
	std::cout << q5 << std::endl << std::endl;

	q6a = (float)(asin((A/C)*sin(M_PI - q5)));
	q7 = (float)atan2(r(1), r(2));						// 足首のロール角
	if (q7 > M_PI/2.0) {
		q7 -= (float)M_PI;
	} else if (q7 < -M_PI/2.0) {
		q7 += (float)M_PI;
	}
	q6 = -atan2(r(0), sign(r(2)) * sqrt(r(1) * r(1) + r(2) * r(2))) - q6a;
	R = Body->R.transpose() * RFoot->R * rodrigues(Rroll, -q7) * rodrigues(Rpitch, -q6-q5);
	q2 = atan2(-R(0,1), R(1,1));
	cz = cos(q2); sz = sin(q2);
	q3 = atan2(R(2,1), -R(0,1)*sz + R(1,1)*cz);
	q4 = atan2(-R(2,0), R(2,2));
	qp = q4 + q5 + q6;

	angle[Const::FOOT_ROLL_R] = q7;
	angle[Const::KNEE_R1    ] = q4 + q5;
	angle[Const::KNEE_R2    ] = q4 - qp;
	angle[Const::LEG_PITCH_R] = qp;
	angle[Const::LEG_ROLL_R ] = q3;
	angle[Const::LEG_YAW_R  ] = q2;
}

#endif

template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const float &tolerance=1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
  svd_opt= ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for(long i=0; i<sigma.size(); ++i)
  {
    if(sigma(i) > tolerance)
      sigma_inv(i)= 1.0/sigma(i);
    else
      sigma_inv(i)= 0.0;
  }
  return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
}

/*!
 * @brief 逆運動学を計算するための関数
 */
void Kinematics::InverseKinematics(vector<int> to, vector<Link> target)
{
	const float EPS = 1.0e-6;
	Matrix3f J;

	ForwardKinematics(Const::CC);
	std::vector<std::vector<int> > idx;
	for(int i = 0; i < to.size(); i ++){
		idx.push_back(FindRoute(to[i]));
	}

	vector<bool> is_finish;
	for(int i = 0; i < to.size(); i ++) is_finish.push_back(false);

	for(int i = 0; i < 100; i ++){
		int finish_count = 0;
		for(int j = 0; j < to.size(); j ++){
			if (is_finish[j]){
				finish_count ++;
				continue;
			}
			MatrixXf J = CalcJacobian(idx[j]);
			MatrixXf err = CalcVWerr(target[j], link[to[j]]);

			if (err.norm() < EPS){
				is_finish[j] = true;
				finish_count ++;
	            std::cout << "finish id: " << j << ", num: " << i << std::endl;
				continue;
			}

#if 0
			// LM method (QR decomposition)
			const float lambda = 0.001f;
			const auto Hk = J.transpose() * J + lambda * Eigen::MatrixXf::Identity(err.size(), err.size());
			const auto gk = J.transpose() * err;
			VectorXf dq = Hk.colPivHouseholderQr().solve(gk);
#endif
#if 0
			// LM method
			const float lambda = 0.001f;
			const auto Hk = J.transpose() * J + lambda * Eigen::MatrixXf::Identity(err.size(), err.size());
			const auto gk = J.transpose() * err;
			VectorXf dq = Hk.inverse() * gk;
#endif
#if 1
			// Newton Raphson (QR decomposition)
			const float lambda = 0.7;
			VectorXf dq = lambda * J.colPivHouseholderQr().solve(err);
#endif
#if 0
			// Newton Raphson
			const float lambda = 0.7;
			VectorXf dq = lambda * (J.inverse() * err);
#endif

			for(int nn = 0; nn < idx[j].size(); nn ++){
				int k = idx[j].at(nn);
				link[k].q = link[k].q + dq(nn);
			}
		}
		if (finish_count == to.size()) break;
		link[Const::RP2].q = - link[Const::RP1].q;		// 平行リンクのために追加した処理
		link[Const::LP2].q = - link[Const::LP1].q;
		ForwardKinematics(Const::CC);
	}
}

/*
 * @brief 胴体から目標リンクへ至る経路
 */
std::vector<int> Kinematics::FindRoute(int n)
{
	std::vector<int> ret;
	int i = n;

	while(i != Const::CC){
		if ((i != Const::RP2)&&(i != Const::LP2))
			ret.push_back(i);
		i = link[i].mother;
	}
	reverse(ret.begin(), ret.end());
	return ret;
}


/*
 * @brief 位置と姿勢の誤差を計算する関数
 */
VectorXf Kinematics::CalcVWerr(Link Cref, Link Cnow)
{
	VectorXf err(6);
	
	Vector3f perr = Cref.p - Cnow.p;
	Matrix3f Rerr = Cnow.R.inverse() * Cref.R;
	Vector3f werr = Cnow.R * rot2omega(Rerr);
	err(0) = perr(0); err(1) = perr(1); err(2) = perr(2);
	err(3) = werr(0); err(4) = werr(1); err(5) = werr(2);

	return err;
}

/*
 * @brief 行列対角関数
 */
Vector3f Kinematics::rot2omega(Matrix3f rot)
{
	using ::std::numeric_limits;

	const float EPS = 1.0e-6;
	Vector3f ret;
	float t, s, a, b;
	
	a = (rot(0,0) + rot(1,1) + rot(2,2) - 1.0f) / 2.0f;
	if (fabs(a - 1) < EPS){
		ret << 0,0,0;
	} else {
		t = acos(min(a, 1.0f));
		s = sin(t);
		if (s < numeric_limits<float>::epsilon()){
			ret << 0,0,0;
		} else {
			b = t / (2.0f * s);
			ret(0) = b * (rot(2,1) - rot(1,2));
			ret(1) = b * (rot(0,2) - rot(2,0));
			ret(2) = b * (rot(1,0) - rot(0,1));
		}
	}
	return ret;
}

/*
 * @brief ヤコビアンの計算
 */
MatrixXf Kinematics::CalcJacobian(std::vector<int> idx)
{
	int jsize = idx.size();
	Vector3f target = link[idx.back()].p;
	MatrixXf J = MatrixXf::Zero(6, jsize);
	for(int i = 0; i < jsize; i ++){
		int j = idx.at(i);
		Vector3f a = link[j].R * link[j].a;
		Vector3f b = a.cross(target - link[j].p);
		J(0,i) = b(0); J(1,i) = b(1); J(2,i) = b(2);
		if ((j != Const::RP1)&&(j != Const::LP1)&&(j != Const::RP3)&&(j != Const::LP3)){			// 平行リンクのために追加した処理
			J(3,i) = a(0); J(4,i) = a(1); J(5,i) = a(2);
		} else {
			J(3,i) = 0; J(4,i) = 0; J(5,i) = 0;
		}
	}
	
	return J;
}

/*!
 * @brief 逆順運動学の計算
 *
 * @param[out] angle 関節の角度列(rad)（サーボモータのIDに対応するデータ）
 * @param[in] RFLink 右足先の位置・姿勢
 * @param[in] LFLink 左足先の位置・姿勢
 */
void Kinematics::calcInverseKinematics(float *angle, Link RFLink, Link LFLink)
{
	float q2, q3, q4, q4a, q5, q6, q7;
	vector<int> to;
	vector<Link> target;

	to.push_back(Const::RR2);			// 右足の設定
	target.push_back(RFLink);
	
	to.push_back(Const::LR2);			// 左足の設定
	target.push_back(LFLink);

	InverseKinematics(to, target);		// 逆運動学の計算
	
	angle[Const::ANKLE_ROLL_R ] =  link[Const::RR2].q                       ;	// 足首ロール軸
	angle[Const::ANKLE_PITCH_R] =  link[Const::RP3].q + link[Const::RP4].q  ;	// 足首ピッチ軸
	angle[Const::SHIN_PITCH_R ] =  link[Const::RP3].q;	// 足首ピッチ軸
	angle[Const::THIGH_PITCH_R] =  link[Const::RP1].q						;	// 膝上ピッチ軸
	angle[Const::HIP_ROLL_R   ] =  link[Const::RR1].q						;	// 股ロール軸
	angle[Const::HIP_YAW_R    ] =  link[Const::RY ].q						;	// 股ヨー軸

	angle[Const::ANKLE_ROLL_L ] =  link[Const::LR2].q                       ;	// 足首ロール軸
	angle[Const::ANKLE_PITCH_L] =  link[Const::LP3].q + link[Const::LP4].q  ;	// 足首ピッチ軸
	angle[Const::SHIN_PITCH_L ] =  link[Const::LP3].q;	// 足首ピッチ軸
	angle[Const::THIGH_PITCH_L] =  link[Const::LP1].q						;	// 膝上ピッチ軸
	angle[Const::HIP_ROLL_L   ] =  link[Const::LR1].q						;	// 股ロール軸
	angle[Const::HIP_YAW_L    ] =  link[Const::LY ].q						;	// 股ヨー軸
}

#if 1

// チェックプログラム
int main()
{
	class Link link[Const::LINK_NUM];
	class Kinematics kine(link);
	float servo_angle[Const::SERVO_MAX_ID];

	initLink(link);
	
	for(int i = 0; i < Const::SERVO_MAX_ID; i ++){
		servo_angle[i] = 0;
	}
	servo_angle[Const::ANKLE_ROLL_R ] =  M_PI / 8.0;
	servo_angle[Const::ANKLE_PITCH_R] =  M_PI / 8.0;
	servo_angle[Const::SHIN_PITCH_R ] =  M_PI / 4.0;
	servo_angle[Const::THIGH_PITCH_R] = -M_PI / 4.0;
	servo_angle[Const::HIP_ROLL_R   ] =  M_PI / 8.0;
	servo_angle[Const::HIP_YAW_R    ] =  M_PI / 8.0;

//	servo_angle[Const::ANKLE_ROLL_L ] =  M_PI / 8.0;
//	servo_angle[Const::ANKLE_PITCH_L] =  M_PI / 8.0;
//	servo_angle[Const::SHIN_PITCH_L ] =  M_PI / 4.0;
//	servo_angle[Const::THIGH_PITCH_L] = -M_PI / 4.0;
//	servo_angle[Const::HIP_ROLL_L   ] = -M_PI / 8.0;
//	servo_angle[Const::HIP_YAW_L    ] =  M_PI / 8.0;

	// 順運動学の計算
	kine.setJointAngle(servo_angle);
	kine.calcForwardKinematics();

	// 保存
	Link RFLink = link[Const::RR2];
	Link LFLink = link[Const::LR2];

	std::cout << "RIGHT FOOT:\n" << link[Const::RR2].p << std::endl << std::endl;
	std::cout << link[Const::RR2].R << std::endl << std::endl;
	std::cout << "LEFT FOOT:\n" << link[Const::LR2].p << std::endl << std::endl;
	std::cout << link[Const::LR2].R << std::endl << std::endl;

	// 関節角度の変更
	servo_angle[Const::ANKLE_ROLL_R ] =  0.0;
	servo_angle[Const::ANKLE_PITCH_R] =  0.0;
	servo_angle[Const::SHIN_PITCH_R ] =  M_PI / 8.0;
	servo_angle[Const::THIGH_PITCH_R] = -M_PI / 8.0;
	servo_angle[Const::HIP_ROLL_R   ] =  0.0;
	servo_angle[Const::HIP_YAW_R    ] =  0.0;
    
	// 足は曲がっていることが必須
	servo_angle[Const::SHIN_PITCH_L ] =  M_PI / 8.0;
	servo_angle[Const::THIGH_PITCH_L] = -M_PI / 8.0;

	kine.setJointAngle(servo_angle);
	kine.calcForwardKinematics();

	// 逆運動学の計算
	kine.calcInverseKinematics(servo_angle, RFLink, LFLink);
	kine.setJointAngle(servo_angle);
	kine.calcForwardKinematics();

	std::cout << "Inverse Kinematics\nRIGHT FOOT:\n" << link[Const::RR2].p << std::endl << std::endl;
	std::cout << link[Const::RR2].R << std::endl << std::endl;

	std::cout << "ANKLE_ROLL_R :" << servo_angle[Const::ANKLE_ROLL_R ] << std::endl;
	std::cout << "ANKLE_PITCH_R:" << servo_angle[Const::ANKLE_PITCH_R] << std::endl;
	std::cout << "SHIN_PITCH_R :" << servo_angle[Const::SHIN_PITCH_R ] << std::endl;
	std::cout << "THIGH_PITCH_R:" << servo_angle[Const::THIGH_PITCH_R] << std::endl;
	std::cout << "HIP_ROLL_R   :" << servo_angle[Const::HIP_ROLL_R   ] << std::endl;
	std::cout << "HIP_YAW_R    :" << servo_angle[Const::HIP_YAW_R    ] << std::endl << std::endl;

	std::cout << "Inverse Kinematics\nLEFT FOOT:\n" << link[Const::LR2].p << std::endl << std::endl;
	std::cout << link[Const::LR2].R << std::endl << std::endl;

	std::cout << "ANKLE_ROLL_L :" << servo_angle[Const::ANKLE_ROLL_L ] << std::endl;
	std::cout << "ANKLE_PITCH_L:" << servo_angle[Const::ANKLE_PITCH_L] << std::endl;
	std::cout << "SHIN_PITCH_L :" << servo_angle[Const::SHIN_PITCH_L ] << std::endl;
	std::cout << "THIGH_PITCH_L:" << servo_angle[Const::THIGH_PITCH_L] << std::endl;
	std::cout << "HIP_ROLL_L   :" << servo_angle[Const::HIP_ROLL_L   ] << std::endl;
	std::cout << "HIP_YAW_L    :" << servo_angle[Const::HIP_YAW_L    ] << std::endl << std::endl;

	return 0;
}

#endif
