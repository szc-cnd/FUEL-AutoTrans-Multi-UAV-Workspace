#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct AStarSearchRegion
{
	bool enabled{false};
	bool limit_max_z{false};
	bool limit_corridor{false};
	bool limit_side{false};
	bool limit_side_band{false};
	bool limit_local_max_z{false};
	double max_z{0.0};
	double corridor_half_width{0.0};
	double side_half_length{0.0};
	double side_band_center{0.0};
	double side_band_half_width{0.0};
	double height_half_length{0.0};
	double time_limit{0.2};
	bool report_timeout{true};
	Eigen::Vector3d line_start{Eigen::Vector3d::Zero()};
	Eigen::Vector3d line_end{Eigen::Vector3d::Zero()};
	Eigen::Vector3d side_origin{Eigen::Vector3d::Zero()};
	Eigen::Vector3d side_normal{Eigen::Vector3d::Zero()};
	Eigen::Vector3d side_axis{Eigen::Vector3d::Zero()};
	Eigen::Vector3d height_origin{Eigen::Vector3d::Zero()};
	Eigen::Vector3d height_axis{Eigen::Vector3d::Zero()};
};

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos) { return grid_map_->getInflateOccupancy(pos); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);
	bool insideSearchRegion(const Eigen::Vector3d &point,
							const AStarSearchRegion *region) const;

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt,
						  Eigen::Vector3d end_pt,
						  const AStarSearchRegion *region = nullptr);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
