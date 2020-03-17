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
	bool operator()(const Node_pointer& left, const Node_pointer& right)const  //���ڣ���������
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
};//�Զ���set����׼�����Ȱ�cost���򣬴��Ű�x����x��ķ��ڿ�ʼ��
using Node_set=set<Node_pointer,sort_Node>;

struct obstacle {
	//�ϰ�������
	//�ϰ�����·��Լ��������˫�򷨱�ʾ
	float x_c,y_c,R_c;
	float vx,vy;
	obstacle() = default;
};
using obs_set=set<obstacle*>; 
/*class path_side {
public:
	//·������
	//�ڴ˽�����·��
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
	float dis_len = 0.5;//�����ڳ�βԲ�ĵ���ͷԲ�ľ���
	float R_car = 1;//����Բ�İ뾶
};