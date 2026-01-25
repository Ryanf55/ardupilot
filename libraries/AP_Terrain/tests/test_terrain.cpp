#include <AP_gtest.h>

#include "AP_Terrain/AP_Terrain.h"
#include "AP_Common/Location.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(AP_Terrain, basic)
{
    AP_Terrain terrain;

    EXPECT_TRUE(terrain.enabled());

    Location loc;
    AP_Terrain::grid_info ginfo;

    terrain.calculate_grid_info(loc, ginfo);

    EXPECT_EQ(ginfo.lat_degrees, 0);
    EXPECT_EQ(ginfo.lon_degrees, 0);
    EXPECT_EQ(ginfo.grid_lat, 0);
    EXPECT_EQ(ginfo.grid_lon, 0);


    loc.lat = 40.0 * 1e7;
    loc.lng = (-105 * 1e7 ) - 1;

    terrain.calculate_grid_info(loc, ginfo);

    EXPECT_EQ(ginfo.lat_degrees, 40);
    EXPECT_EQ(ginfo.lon_degrees, -106);

    EXPECT_TRUE(ginfo.grid_lat* 1e-7 <= loc.lat);


}

AP_GTEST_MAIN()
