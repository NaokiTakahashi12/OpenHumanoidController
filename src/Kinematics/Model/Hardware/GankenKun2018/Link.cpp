/*!
 * @file Link.cpp
 * @brief リンクのデータ
 * @date 2014.1.2
 * @author Yasuo Hayashibara
 */

#include <math.h>
#include "Constant.h"
#include "Link.h"

/*
 * @brief リンクの初期化
 */
void GankenKun2018::initLink(class Link *link)
{
	for(int i = 0; i < Const::LINK_NUM; i ++){
		float p[3];
		for(int j = 0; j < 3; j ++)
			p[j] = Const::LINK_PARA[i].p[j] / 1000.0f;		// リンクの位置 (mm -> m)
		link[i].setAxisPos(Const::LINK_PARA[i].a, p);
		link[i].setConnection(Const::LINK_CONNECT[i].sister, Const::LINK_CONNECT[i].child, Const::LINK_CONNECT[i].mother);
															// リンクの接続
	}
}
