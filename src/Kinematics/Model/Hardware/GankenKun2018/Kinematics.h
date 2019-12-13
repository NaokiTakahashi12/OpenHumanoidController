/*!
 * @file Kinematics.h
 * @brief 運動学を計算するプログラム
 * @date 2014.1.1
 * @author Yasuo Hayashibara
 */

#pragma once

#include "Link.h"
#include <vector>

/*!
 * @class Kinematics
 * @brief 運動学を計算するプログラム
 */
namespace GankenKun2018 {

class Kinematics
{
	class Link *link;

public:
	/*!
	 * @brief コンストラクタ
	 * データを初期化する
	 *
	 * @param[in] link 設定するリンクデータ
	 */
	Kinematics(class Link *link);

	/*!
	 * @brief 順運動学の計算
	 */
	void calcForwardKinematics();

	/*
	 * @brief モータの角度の設定
	 *
	 * @param[in] angle 関節の角度列(rad)（サーボモータのIDに対応するデータ）
	 */
	void setJointAngle(float *angle);

	/*!
	 * @brief 逆順運動学の計算
	 *
	 * @param[out] angle 関節の角度列(rad)（サーボモータのIDに対応するデータ）
	 * @param[in] RFLink 右足先の位置・姿勢
	 * @param[in] LFLink 左足先の位置・姿勢
	 */
	void calcInverseKinematics(float *angle, Link RFLink, Link LFLink);

private:
	/*!
	 * @brief 順運動学を再帰的に計算するための関数
	 */
	void ForwardKinematics(int n);

	/*!
	 * @brief 逆運動学を計算するための関数
	 *
	 * @param[in] to 位置・姿勢を計算するリンク番号の列
	 * @param[in] target リンクの位置・姿勢の目標の列
	 */
	void InverseKinematics(vector<int> to, vector<Link> target);

	/*
	 * @brief 胴体から目標リンクへ至る経路
	 */
	std::vector<int> FindRoute(int n);
	
	/*
	 * @brief 位置と姿勢の誤差を計算する関数
	 */
	VectorXf CalcVWerr(Link Cref, Link Cnow);

	/*
	 * @brief 行列対角関数
	 */
	Vector3f rot2omega(Matrix3f rot);

	/*
	 * @brief ヤコビアンの計算
	 */
	MatrixXf CalcJacobian(std::vector<int> idx);
};

}
