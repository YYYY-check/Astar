#include<set>
#include<vector>
#include<list>
#include <algorithm>
using namespace std;
struct point
{
	float x, y;
	bool operator==(const point& coor_xy) {
		return((x == coor_xy.x) & (y == coor_xy.y));
	}
};
struct Node
{
	point coordinate;
	float g, h;
	Node* father;
	Node() = default;
	Node(point coordinate_, Node* father_ = nullptr) {
		coordinate = coordinate_;
		father = father_;
		g = 0;
		h = 100000;
	}
	float cost() {
		return g + h;
	};
};
using Node_pointer=Node *;
struct sort_Node {
	bool operator()(const Node_pointer& left, const Node_pointer& right)const  //对于（）的重载
	{
		if (left->cost() < right->cost()) {return true;}
		else if (left->cost() == right->cost()) {
			if (left->coordinate.x > right->coordinate.x) { return true; }
			else if (left->coordinate.x == right->coordinate.x) {
				if (left->coordinate.y > right->coordinate.y) { return true; }
				else { return false; }
			}
			else { return false; }
		}
		else { return false; }
		//return (left->cost() < right->cost());
	}
};//自定义set排序准则；优先按cost排序，次优按x排序x大的放于开始处
using Node_set=set<Node_pointer,sort_Node>;

struct obstacle {
	//障碍物数量
	//障碍车与路边约束均采用双球法表示
	float x_c,y_c,R_c;
	float vx,vy;
	obstacle() = default;
};
using obs_set=set<obstacle*>; 
/*class path_side {
public:
	//路边数据
	//在此仅考虑路宽
	float path_l;
	float ;
	float vx, vy;
	obstacle() = default;
};*/
class Astar {
public:
	Astar() = default;
	list<float>path_x, path_y;
	bool check_crash(point* new_point, obs_set obs_list, Node* node_father);
	float caculate_cost_g(point* new_point, point* end_point, Node* node_father);
	float caculate_cost_h(point* new_point, point* end_point, Node* node_father);
	void update_vertices(Node_set& openlist_, set<Node_pointer>& closelist_, Node* new_node, point* end_point);
	void find_path_node(point* start_point, point* end_point, float(*search_derection)[2], obs_set& obs);
private:
	float dis_len = 0.5;//车辆第车尾圆心到车头圆心距离
	float R_car = 1;//车辆圆心半径
};