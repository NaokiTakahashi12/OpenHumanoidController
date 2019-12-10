/*!
 * @file Link.h
 * @brief リンクのデータ
 * @date 2014.1.2
 * @author Yasuo Hayashibara
 */

#pragma once

#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace GankenKun2018 {

class Link {
public:
	string name;		//! リンクの名称
	int sister;			//! 妹リンクID
	int child;			//! 子リンクID
	int mother;			//! 親リンクID
	Vector3f p;			//! ワールド座標系での位置
	Matrix3f R;			//! ワールド座標系での姿勢
	Vector3f v;			//! ワールド座標系での速度
	Vector3f w;			//! ワールド座標系での角速度ベクトル
	float q;
	float dq;
	float ddq;
	Vector3f a;			//! 関節軸ベクトル
	Vector3f b;			//! 相対位置 (m)
	float vartex;
	float face;
	float m;
	float c;
	float I;

	/*
	 * @brief コンストラクタ
	 */
	Link():
		name(""), q(0), dq(0), ddq(0)
	{
		p << 0, 0, 0;
		R = Matrix3f::Identity();
	}
	
	/*
	 * @brief コンストラクタ
	 */
	Link(const Link& obj)
	{
		name	= obj.name		;
		sister	= obj.sister	;
		child	= obj.child		;
		mother	= obj.mother	;
		p		= obj.p			;
		R		= obj.R			;
		v		= obj.v			;
		w		= obj.w			;
		q		= obj.q			;
		dq		= obj.dq		;
		ddq		= obj.ddq		;
		a		= obj.a			;
		b		= obj.b			;
		vartex	= obj.vartex	;
		face	= obj.face		;
		m		= obj.m			;
		c		= obj.c			;
		I		= obj.I			;
	}
	
	/*
	 * @brief リンクの関節軸と相対位置の設定
	 *
	 * @param[in] a[3] リンクの関節軸
	 * @param[in] p[3] リンクの相対位置 (m)
	 */
	void setAxisPos(const float a[3], const float p[3]) {
		for(int i = 0; i < 3; i ++){
			this->a[i] = a[i];		//! リンクの回転軸
			this->b[i] = p[i];		//! リンクの相対距離
		}
	}
	
	void setConnection(int sister, int child, int mother){
		this->sister = sister;
		this->child  = child ;
		this->mother = mother;
	}
};

void initLink(class Link *link);	//! リンクの初期化

}

