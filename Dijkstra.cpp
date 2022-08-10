#include<queue>
#include<iostream>
#include<algorithm>
#include<string.h>
using namespace std;
const int N = 100;//城市的个数可修改
const int INF = 1e7;//初始化无穷大为10000000
int map[N][N], dist[N], p[N], n, m;//n为城市的个数，m为城市间路线的条数
int flag[N];	//	如果flag[i]==true,说明顶点i已经加入到集合S；否则顶点i属于集合V-S

struct Node {
	int u, step;
	Node() {};
	Node(int a, int sp)
	{
		u = a, step = sp;
	}
	bool operator<(const Node& a)const {//重载 <
		return step > a.step;
	}
};

void Dijkstra(int st)
{
	priority_queue<Node>Q;//优先队列优化
	Q.push(Node(st, 0));
	memset(flag, 0, sizeof(flag));//初始化flag数组为0
	for (int i = 1; i <= n; ++i)
		dist[i] = INF;//初始化所有距离为无穷大
	dist[st] = 0;
	while (!Q.empty())
	{
		Node it = Q.top();//优先队列列头元素为最小值,也就是下面for循环压完后，再重新弹出最小的点，再循环
		Q.pop();
		int t = it.u;
		if (flag[t])//说明已经找到了最短距离，该节点是队列里面的重复元素
			continue;
		flag[t] = 1;
		for (int i = 1; i <= n; i++)
		{
			if(!flag[i] && map[t][i]<INF)//判断与当前点有关系的点，并且自己不能到自己
				if (dist[i] > dist[t] + map[t][i])  //******这一句最关键，dist[i]代表i到起点的距离，如果不能到达就是无穷大*****
				{
					//求距离当前点的每个点的最短距离，进行松弛操作
					dist[i] = dist[t] + map[t][i];
					Q.push(Node(i, dist[i]));//把更新后的最短距离压入队列中，注意：里面有重复元素
				}
		}
	}

}

int main()
{
	int u, v, w, st;
	system("color 0d");
	cout << "请输入城市的个数：" << endl;
	cin >> n;
	cout << "请输入城市之间的路线个数" << endl;
	cin >> m;
	cout << "请输入城市之间的路线以及距离" << endl;
	for (int i = 1; i <= n; i++)//初始化图的邻接矩阵
		for (int j = 1; j <= n; j++)
		{
			map[i][j] = INF;//初始化邻接矩阵为无穷大
		}
	while (m--)
	{
		cin >> u >> v >> w;
		map[u][v] = min(map[u][v], w);	//邻接矩阵存储，保留最小的距离
	}
	cout << "请输入小明所在的位置：" << endl;
	cin >> st;
	Dijkstra(st);
	cout << "小明所在的位置：" << st << endl;
	for (int i = 1; i <= n; i++)
	{
		cout << "小明：" << st << " ---> " << "要去的位置：" << i;
		if (dist[i] == INF)
			cout << "sorry,无路可达" << endl;
		else
			cout << " 最短距离为：" << dist[i] << endl;
	}
	return 0;
}
/*
请输入城市的个数：
5
请输入城市之间的路线个数
11
请输入城市之间的路线以及距离
1 5 2
5 1 8
1 2 16
2 1 29
5 2 32
2 4 13
4 2 27
1 3 15
3 1 21
3 4 7
4 3 19
请输入小明所在的位置：
5
小明所在的位置：5
小明：5 ---> 要去的位置：1 最短距离为：8
小明：5 ---> 要去的位置：2 最短距离为：24
小明：5 ---> 要去的位置：3 最短距离为：23
小明：5 ---> 要去的位置：4 最短距离为：30
小明：5 ---> 要去的位置：5 最短距离为：0
*/
