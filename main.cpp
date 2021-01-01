#include <bits/stdc++.h>

using namespace std;

// store the center of polygon
pair<int, int> mid;

// quadrant of a point
int quad(pair<int, int> p)
{
    if (p.first >= 0 && p.second >= 0) return 1;
    if (p.first <= 0 && p.second >= 0) return 2;
    if (p.first <= 0 && p.second <= 0) return 3;
    return 4;
}

// check whether the line is crossing the polygon
int orientation(pair<int, int> a, pair<int, int> b, pair<int, int> c)
{
    int val = (b.second - a.second) * (c.first - b.first) -
              (c.second - b.second) * (b.first - a.first);

    if (val == 0) return 0;
    if (val > 0) return 1;
    return -1;
}

// compare for sorting
bool compare(pair<int, int> p1, pair<int, int> q1)
{
    pair<int, int> p = make_pair(p1.first - mid.first,
                                 p1.second - mid.second);
    pair<int, int> q = make_pair(q1.first - mid.first,
                                 q1.second - mid.second);

    int one = quad(p);
    int two = quad(q);

    if (one != two) return (one < two);
    return (p.second * q.first < q.second * p.first);
}

// find convex hull for a set of less than 6 points
vector<pair<int, int>> brute_hull(vector<pair<int, int>> points)
{
    set<pair<int, int>> s;

    for (int i = 0; i < points.size(); ++i) {
        for (int j = i + 1; j < points.size(); ++j) {
            int x1 = points[i].first, x2 = points[j].first;
            int y1 = points[i].second, y2 = points[j].second;

            int a1 = y1 - y2;
            int b1 = x2 - x1;
            int c1 = x1 * y2 - y1 * x2;
            int pos = 0, neg = 0;

            for (int k = 0; k < points.size(); ++k) {
                if (a1 * points[k].first + b1 * points[k].second + c1 <= 0)
                {
                    neg++;
                }
                if (a1 * points[k].first + b1 * points[k].second + c1 >= 0)
                {
                    pos++;
                }
            }
            if (pos == points.size() || neg == points.size())
            {
                s.insert(points[i]);
                s.insert(points[j]);
            }
        }
    }

    vector<pair<int, int>> ret;
    for (auto e : s)
    {
        ret.push_back(e);
    }

    // sorting points in anti-clockwise order
    mid = {0, 0};
    int n = ret.size();
    for (int i = 0; i < n; ++i) {
        mid.first += ret[i].first;
        mid.second += ret[i].second;
        ret[i].first *= n;
        ret[i].second *= n;
    }
    sort(ret.begin(), ret.end(), compare);
    for (int i = 0; i < n; ++i) {
        ret[i] = make_pair(ret[i].first/n, ret[i].second/n);
    }

    return ret;
}

// find upper tangent of two polygons
vector<pair<int, int>> merger(vector<pair<int, int>> a,
                              vector<pair<int, int>> b)
{
    int n1 = a.size(), n2 = b.size();

    int ia = 0, ib = 0;
    for (int i = 1; i < n1; ++i) {
        if (a[i].first > a[ia].first)
        {
            ia = i;
        }
    }

    for (int i = 1; i < n2; ++i) {
        if (b[i].first < b[ib].first)
        {
            ib = i;
        }
    }

    // finding upper tangent
    int inda = ia, indb = ib;
    bool done = false;
    while (!done)
    {
        done = true;
        while (orientation(b[indb], a[inda], a[(inda + 1) % n1]) >= 0)
        {
            inda = (inda + 1) % n1;
        }
        while (orientation(a[inda], b[indb], b[(n2 + indb - 1) % n2]) <= 0)
        {
            indb = (n2 + indb - 1) % n2;
            done = false;
        }
    }

    // finding lower tangent
    int uppera = inda, upperb = indb;
    inda = ia, indb = ib;
    done = false;
    while (!done)
    {
        done = true;
        while (orientation(a[inda], b[indb], b[(indb + 1) % n2]) >= 0)
        {
            indb = (indb + 1) % n2;
        }
        while (orientation(b[indb], a[inda], a[(n1 + inda - 1) % n1]) <= 0)
        {
            inda = (n1 + inda - 1) % n1;
            done = false;
        }
    }

    int lowera = inda, lowerb = indb;
    vector<pair<int, int>> ret;

    // ret contains the convex hull after merging the two convex hulls
    // with the points sorted in anti-clockwise order
    int ind = uppera;
    ret.push_back(a[uppera]);
    while (ind != lowera)
    {
        ind = (ind + 1) % n1;
        ret.push_back(a[ind]);
    }

    ind = lowerb;
    ret.push_back(b[lowerb]);
    while (ind != upperb)
    {
        ind = (ind + 1) % n2;
        ret.push_back(b[ind]);
    }

    return ret;
}

// convex hull for given set of points
vector<pair<int, int>> find_hull(vector<pair<int, int>> points)
{
    // number of points is less than 6
    // find the convex hull
    if (points.size() <= 5) return brute_hull(points);

    // left contains the left half points
    // right contains the right half points
    vector<pair<int, int>> left, right;
    for (int i = 0; i < points.size()/2; i++) {
        left.push_back(points[i]);
    }
    for (int i = points.size()/2; i < points.size(); i++) {
        right.push_back(points[i]);
    }

    // convex hull
    vector<pair<int, int>> left_hull = find_hull(left);
    vector<pair<int, int>> right_hull = find_hull(right);

    // merging convex hulls
    return merger(left_hull, right_hull);
}

// return convex hull after removing a point
vector<pair<int, int>> remove_point(vector<pair<int, int>> points,
                                    vector<pair<int, int>> hull,
                                    pair<int, int> p)
{
    // checking whether point is a part of convex hull or not
    bool found = 0;
    for (int i = 0; i < hull.size() && !found; ++i) {
        if (hull[i].first == p.first &&
            hull[i].second == p.second)
        {
            found = 1;
        }
    }

    // if point is not part of convex hull
    if (found == 0) return hull;

    // remove the point and again make the convex hull
    for (int i = 0; i < points.size(); ++i) {
        if (points[i].first == p.first && points[i].second == p.second)
        {
            points.erase(points.begin() + i);
            break;
        }
    }

    sort(points.begin(), points.end());
    return find_hull(points);
}

int main() {
    vector<pair<int, int>> points;
    points.push_back(make_pair(0, 0));
    points.push_back(make_pair(1, -4));
    points.push_back(make_pair(-1, -5));
    points.push_back(make_pair(-5, -3));
    points.push_back(make_pair(-3, -1));
    points.push_back(make_pair(-1, -3));
    points.push_back(make_pair(-2, -2));
    points.push_back(make_pair(-1, -1));
    points.push_back(make_pair(-2, -1));
    points.push_back(make_pair(-1, 1));

    int n = points.size();

    // sorting set of points according
    // to the x-coordinate
    sort(points.begin(), points.end());
    vector<pair<int, int>> hull = find_hull(points);

    cout << "Convex hull:\n";
    for (auto e : hull) {
        cout << e.first << " " << e.second << endl;
    }

    pair<int, int> p = make_pair(-5, -3);
    vector<pair<int, int>> mod_hull = remove_point(points, hull, p);


    cout << "Modified Convex hull:\n";
    for (auto e : mod_hull) {
        cout << e.first << " " << e.second << endl;
    }

    return 0;
}
