import math

def calculate_distance_bearing(lat1, lon1, lat2, lon2):
    """Test the bearing calculation with your mission coordinates"""
    
    R = 6371000  # Earth radius (m)
    lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
    lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    # Distance calculation
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    # Bearing calculation
    if abs(dlat) < 1e-10 and abs(dlon) < 1e-10:
        bearing = 0.0
    else:
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
        
        bearing_rad = math.atan2(y, x)
        bearing = math.degrees(bearing_rad)
        bearing = (bearing + 360) % 360
    
    return distance, bearing

# Test with your actual mission coordinates
start_lat = 34.25739
start_lon = -117.1987
waypoint_lat = 34.25738998077238
waypoint_lon = -117.20087617167137

print("=== MISSION BEARING TEST ===")
print(f"Start: ({start_lat}, {start_lon})")
print(f"Waypoint: ({waypoint_lat}, {waypoint_lon})")

distance, bearing = calculate_distance_bearing(start_lat, start_lon, waypoint_lat, waypoint_lon)

print(f"Distance: {distance:.1f}m")
print(f"Bearing: {bearing:.1f}°")
print(f"Expected bearing: ~270° (due west)")

# Check if bearing is correct
if 260 <= bearing <= 280:
    print("✓ Bearing calculation looks correct")
else:
    print("✗ Bearing calculation may be wrong")

# Test a few points along the path to see if bearing changes
print("\n=== BEARING STABILITY TEST ===")
test_positions = [
    (34.25739, -117.1990),    # Partway to waypoint
    (34.25739, -117.1995),    # Closer to waypoint  
    (34.25739, -117.2005),    # Past waypoint
]

for i, (test_lat, test_lon) in enumerate(test_positions):
    dist, bear = calculate_distance_bearing(test_lat, test_lon, waypoint_lat, waypoint_lon)
    print(f"Test {i+1}: From ({test_lat}, {test_lon}) -> Bearing: {bear:.1f}°")