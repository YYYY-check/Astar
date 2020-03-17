#include"Astar.h"
#include <iostream>
#include<ctime>
#define random(x,y) rand()%(int)(y-x)+x
using namespace std;
bool Astar::check_crash(point* new_point, obs_set obs_list,Node* node_father) {
	//仅考虑主车为一个圆，多个圆模型后续更新
	bool res = true;
	for (auto obs : obs_list) {
		if ((abs(new_point->x - obs->x_c) >= obs->R_c + R_car) || (abs(new_point->y - obs->y_c) >= obs->R_c + R_car)) continue;
		else if (pow(new_point->x - obs->x_c, 2) + pow(new_point->y - obs->y_c, 2) >= pow(R_car + obs->R_c, 2)) continue;
		else {
			res = false;
			break;
		}
	}
	return res;
}
float Astar::caculate_cost_g(point* new_point, point* end_point,Node*node_father) {
	return node_father->g+pow(pow(new_point->x - node_father->coordinate.x, 2) + pow(new_point->y - node_father->coordinate.y, 2), 0.5);
		//abs(new_point->x - node_father->coordinate.x) + abs(new_point->y - node_father->coordinate.y);
}
float Astar::caculate_cost_h(point* new_point, point* end_point, Node* node_father) {
	return abs(new_point->x - end_point->x) + abs(new_point->y - end_point->y);
}
void Astar::update_vertices(Node_set&openlist_, set<Node_pointer>&closelist_, Node*new_node,point*end_point) {
	bool res_close = true;
	bool res_open = true;
	float G_new=0, H_new=0;
	for (auto list : closelist_) {
		if(list->coordinate== new_node->coordinate){ //存在于closelist中的点舍去
			res_close = false;
			break;
		}
	}
	if (res_close) {
		for (Node_set::iterator it = openlist_.begin(); it != openlist_.end();it++) {
			if ((*it)->coordinate == new_node->coordinate) {
				if (new_node->cost() < (*it)->cost()) {//存在于openlist且损失比原有小更新原有父节点
					openlist_.erase(it);
					break;
				}
				else { res_open = false; }
			}
		}
		if (res_open) {
			openlist_.insert(new_node);
		}
	}
};
void Astar::find_path_node(point* start_point, point* end_point, float(*search_derection)[2], obs_set& obs) {
	list<Node>search_Node;//存储所有搜索过的数据，其他均为指针运算
	Node Node_best(*start_point);//存储每次损失最小的节点
	point Pnew;//存储每次新增的point
	Node Nonew;//存储每次新增node
	Node* No_father=nullptr;
	search_Node.push_back(Node_best);
	Node_set openlist;//根据cost大小进行排序
	set<Node*>closelist;//根据指针大小进行排序方便回溯到初始点
	openlist.insert(&search_Node.back());
	while (!openlist.empty()) {
		Node_best = *(*openlist.begin());
		closelist.insert(*openlist.begin());
		No_father = *openlist.begin();
		if (Node_best.coordinate == *end_point) { break; }
		openlist.erase(openlist.begin());//将最佳节点放入closelist后删除openlist对应的节点
		for (int i = 0; i < 8; i++) {
			Pnew.x = Node_best.coordinate.x + search_derection[i][0];
			Pnew.y = Node_best.coordinate.y + search_derection[i][1];
			Nonew.coordinate = Pnew, Nonew.father = No_father;
			Nonew.g = caculate_cost_g(&Pnew, end_point, Nonew.father);
			Nonew.h = caculate_cost_h(&Pnew, end_point, Nonew.father);
			if (check_crash(&Pnew, obs, Nonew.father)) {
				search_Node.push_back(Nonew);
				update_vertices(openlist, closelist,&search_Node.back(),end_point);
			}
		}
		if (Nonew.father == *openlist.begin()) {
			break;
		}
	}
	//set<Node_pointer>::iterator it=closelist.find(No_father);//
	Node*it = No_father;
	do{
		path_x.push_back(it->coordinate.x);
		path_y.push_back(it->coordinate.y);
		it = it->father;
		//it = find(closelist.begin(),closelist.end(), (*it)->father);
	} while (it->father!=nullptr); 
	search_Node.clear();
	openlist.clear();
	closelist.clear();
}