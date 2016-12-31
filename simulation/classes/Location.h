/*
 * Header file providing class for location
 * Author: Frederick Wachter
 */

#ifndef __LOCATION__
#define __LOCATION__

using namespace std;

class Location {
	public:
		int x = 0;
		int y = 0;

		// Defualt constructor
		Location() {}

		// Alternate constructor(s)
		Location(const int x, const in y) {
			this->x = x;
			this->y = y;
		}

		// Member function(s)
		void setLocation(const int x, const int y) {
			this->x = x;
			this->y = y;
		}

		// Overloaded operators
		bool operator == (const Location &rhs) {
			if ((x == rhs.x) && (y == rhs.y)) {
				return true;
			} else {
				return false;
			}
		}
};

#endif


