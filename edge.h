#ifndef EDGE_H
#define EDGE_H

template <class T, class W>
class Edge {
	public:
	T u, v;
	W w;
	Edge(){}
	Edge(T _u, T _v, int _w){
		u = _u;
		v = _v;
		w = _w;
	}
	W getWeight(){
		return w;
	}
	bool operator==(const Edge<T, W>& _e) const {
		return _e.u == u && _e.v == v && _e.w == w;
	}
};

#endif
