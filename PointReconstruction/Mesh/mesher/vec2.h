#pragma once
template<typename T>
class vec2 {
	vector<T> data;
	//vector<T> operator + (vector<T> a, vector<T> b) const;
	//{
	//	return { a[0] + b[0],a[1] + b[1] };
	//}
	vector<T> operator - (vector<T> b) const
	{
		return {data[0] - b[0],data[1] - b[1] };
	}
	//vector<T> operator / (vector<T> a, T b) const;
	//{
	//	return {};
	//}
	//vector<T> operator ** (vector<T> a, vector<T> b) const;
	//{
	//	return { a[0] * b[0] + a[1] * b[1] };
	//}
	//vector<T> operator * (vector<T> a, vector<T> b) const;
	//{
	//	return { a[0] * b[1] - b[0] * a[1] };
	//}

};
 
template<typename T>
T cro(vector<T> a, vector<T> b)
{
	return { abs(a[0] * b[1] - b[0] * a[1])};
}

template<typename T>
T dot(vector<T> a, vector<T> b)
{
	return { a[0] * b[0] + a[1] * b[1] };
}

template<typename T>
vector<T> add(vector<T> a, vector<T> b)
{
	return { a[0] + b[0] , a[1] + b[1] };
}
template<typename T>

vector<T> sub(vector<T> a, vector<T> b)
{
	return { a[0] - b[0] , a[1] - b[1] };
}

template<typename T>
vector<T> scale(vector<T> a, T b)
{
	return { a[0] * b , a[1] * b };
}
template<typename T>
T len(vector<T> a)
{
	return a[0] * a[0] + a[1] * a[1];
}