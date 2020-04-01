struct location {
	double x;
	double y;
};

const location PICKUP = {2.0, -2.0};
const location DROPOFF = {-4.0, 1.0};

enum HomeRobotState {
	nav2PickUp,
	pickingUp,
	nav2DropOff,
	droppingOff
};
