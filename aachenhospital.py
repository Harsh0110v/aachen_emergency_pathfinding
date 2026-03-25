import heapq
import math
import time
import tracemalloc
class CityMap:
    def __init__(self):
        self.coords = {}          
        self.roads = {}           
        self.hospitals = []      

    def add_place(self, name, lat, lon, is_hospital=False):
        self.coords[name] = (lat, lon)
        if name not in self.roads:
            self.roads[name] = []
        if is_hospital:
            self.hospitals.append(name)

    def add_road(self, place_a, place_b, distance_km):

        self.roads[place_a].append((place_b, distance_km))
        self.roads[place_b].append((place_a, distance_km))

    def get_neighbours(self, place):
        return self.roads.get(place, [])

    def straight_line_distance(self, place_a, place_b):

        lat1, lon1 = self.coords[place_a]
        lat2, lon2 = self.coords[place_b]
        R = 6371.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2 +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(dlambda / 2) ** 2)
        return R * 2 * math.asin(math.sqrt(a))

def build_aachen():
    m = CityMap()

    m.add_place("Uniklinik RWTH Aachen", 50.7739, 6.0414, is_hospital=True)
    m.add_place("Luisenhospital Aachen", 50.7784, 6.0966, is_hospital=True)
    m.add_place("St.-Marien-Hospital", 50.7594, 6.1030, is_hospital=True)
    m.add_place("Bethlehem Gesundheitszentrum", 50.7816, 6.1272, is_hospital=True)
    m.add_place("Franziskushospital Aachen", 50.7712, 6.1073, is_hospital=True)
    m.add_place("Laurensberg", 50.791, 6.048)
    m.add_place("Richterich", 50.804, 6.069)
    m.add_place("Eilendorf", 50.774, 6.150)
    m.add_place("Brand", 50.739, 6.106)
    m.add_place("Vaals border", 50.772, 6.016)

    m.add_place("Lousberg", 50.785, 6.058)
    m.add_place("Ponttor", 50.780, 6.080)
    m.add_place("Aachen Hbf", 50.768, 6.092)
    m.add_place("Westbahnhof", 50.768, 6.055)
    m.add_place("City Centre", 50.775, 6.085)
    m.add_place("Marienplatz", 50.778, 6.094)
    m.add_place("Kronenberg", 50.782, 6.127)
    m.add_place("Rothe Erde", 50.771, 6.107)
    m.add_place("Burtscheid", 50.759, 6.103)
    m.add_place("Europaplatz", 50.780, 6.087)
    m.add_place("Kaiserplatz", 50.773, 6.098)
    m.add_place("Elisenbrunnen", 50.776, 6.088)
    m.add_place("Theater", 50.776, 6.092)
    m.add_place("Campus", 50.775, 6.069)

    roads = [

        ("Laurensberg", "Lousberg", 1.2),
        ("Lousberg", "Uniklinik RWTH Aachen", 1.6),


        ("Richterich", "Ponttor", 2.2),
        ("Ponttor", "City Centre", 0.8),
        ("City Centre", "Marienplatz", 0.6),
        ("Marienplatz", "Bethlehem Gesundheitszentrum", 0.8),


        ("Eilendorf", "Kronenberg", 1.8),
        ("Kronenberg", "Bethlehem Gesundheitszentrum", 0.7),


        ("Brand", "Burtscheid", 2.1),
        ("Burtscheid", "St.-Marien-Hospital", 0.3),


        ("Vaals border", "Westbahnhof", 2.5),
        ("Westbahnhof", "Uniklinik RWTH Aachen", 1.7),

        ("Uniklinik RWTH Aachen", "Campus", 0.8),
        ("Campus", "Westbahnhof", 0.9),
        ("Westbahnhof", "Lousberg", 1.2),
        ("Lousberg", "Ponttor", 1.3),
        ("Ponttor", "Aachen Hbf", 1.5),
        ("Aachen Hbf", "Kaiserplatz", 0.7),
        ("Kaiserplatz", "Theater", 0.5),
        ("Theater", "Elisenbrunnen", 0.4),
        ("Elisenbrunnen", "City Centre", 0.3),
        ("City Centre", "Aachen Hbf", 0.9),
        ("City Centre", "Franziskushospital Aachen", 1.2),
        ("Franziskushospital Aachen", "Rothe Erde", 0.5),
        ("Rothe Erde", "Kronenberg", 1.3),
        ("Rothe Erde", "Burtscheid", 2.0),
        ("Burtscheid", "Aachen Hbf", 1.6),
        ("Aachen Hbf", "St.-Marien-Hospital", 1.8),
        ("St.-Marien-Hospital", "Burtscheid", 0.3),
        ("Franziskushospital Aachen", "Bethlehem Gesundheitszentrum", 1.1),
        ("Kaiserplatz", "Marienplatz", 0.4),
        ("Luisenhospital Aachen", "Elisenbrunnen", 0.5),
        ("Elisenbrunnen", "Theater", 0.4),
        ("Theater", "Kaiserplatz", 0.5),
        ("Kaiserplatz", "Aachen Hbf", 0.7),
        ("Europaplatz", "City Centre", 0.5),
        ("Europaplatz", "Kaiserplatz", 0.6),
        ("Europaplatz", "Marienplatz", 0.4),
    ]

    for a, b, d in roads:
        m.add_road(a, b, d)

    return m
def reconstruct_path(prev, goal):
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return path

def dijkstra_to_nearest_hospital(graph, start):
    pq = [(0.0, start)]              
    best_dist = {start: 0.0}
    came_from = {start: None}
    visited = set()
    expanded = 0

    hospitals = set(graph.hospitals)

    while pq:
        cur_dist, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        expanded += 1

        if cur in hospitals:
            return {
                "hospital": cur,
                "distance_km": cur_dist,
                "path": reconstruct_path(came_from, cur),
                "nodes_expanded": expanded,
            }

        for nxt, road_len in graph.get_neighbours(cur):
            new_dist = cur_dist + road_len
            if new_dist < best_dist.get(nxt, math.inf):
                best_dist[nxt] = new_dist
                came_from[nxt] = cur
                heapq.heappush(pq, (new_dist, nxt))

    return {
        "hospital": None,
        "distance_km": math.inf,
        "path": [],
        "nodes_expanded": expanded,
    }


def astar_to_nearest_hospital(graph, start):
    def heuristic(place):

        return min(graph.straight_line_distance(place, h) for h in graph.hospitals)

    open_pq = [(heuristic(start), start)]
    g_score = {start: 0.0}
    came_from = {start: None}
    closed = set()
    expanded = 0

    hospitals = set(graph.hospitals)

    while open_pq:
        _, cur = heapq.heappop(open_pq)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1

        if cur in hospitals:
            return {
                "hospital": cur,
                "distance_km": g_score[cur],
                "path": reconstruct_path(came_from, cur),
                "nodes_expanded": expanded,
            }

        for nxt, road_len in graph.get_neighbours(cur):
            tentative_g = g_score[cur] + road_len
            if tentative_g < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative_g
                came_from[nxt] = cur
                f_val = tentative_g + heuristic(nxt)
                heapq.heappush(open_pq, (f_val, nxt))

    return {
        "hospital": None,
        "distance_km": math.inf,
        "path": [],
        "nodes_expanded": expanded,
    }


def run_algorithm(graph, start, algo_name, algo_func):
    tracemalloc.start()
    t0 = time.perf_counter()
    result = algo_func(graph, start)
    t1 = time.perf_counter()
    _, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    return {
        "origin": start,
        "algorithm": algo_name,
        "hospital": result["hospital"],
        "distance_km": round(result["distance_km"], 3),
        "path": result["path"],
        "nodes_expanded": result["nodes_expanded"],
        "time_ms": round((t1 - t0) * 1000, 4),
        "memory_kb": round(peak_mem / 1024, 2),
    }

def main():
    city = build_aachen()
    scenarios = ["Laurensberg", "Richterich", "Eilendorf", "Brand", "Vaals border"]
    all_results = []

    print("=" * 70)
    print(" Aachen Emergency Pathfinding")
    print(" Dijkstra vs A*")
    print("=" * 70)

    for start in scenarios:
        print(f"\n Starting point: {start}")

        dijk = run_algorithm(city, start, "Dijkstra", dijkstra_to_nearest_hospital)
        astar = run_algorithm(city, start, "A*", astar_to_nearest_hospital)

        all_results.append(dijk)
        all_results.append(astar)

        for res in (dijk, astar):
            print(f"  [{res['algorithm']}] → {res['hospital']}")
            print(f"      Distance: {res['distance_km']} km")
            print(f"      Path: {' → '.join(res['path'])}")
            print(f"      Nodes expanded: {res['nodes_expanded']}")
            print(f"      Time: {res['time_ms']} ms")
            print(f"      Memory: {res['memory_kb']} KB")


        if abs(dijk["distance_km"] - astar["distance_km"]) < 1e-6:
            print("   Validation: both algorithms found the same distance.")
        else:
            print("   Validation: distances differ!")

    print("\n" + "=" * 70)
    print(" SUMMARY")
    print("=" * 70)
    print(f"{'Origin':<16} {'Algorithm':<10} {'Hospital':<32} {'Dist(km)':<9} {'Nodes':<6}")
    print("-" * 80)
    for res in all_results:
        print(f"{res['origin']:<16} {res['algorithm']:<10} "
              f"{res['hospital']:<32} {res['distance_km']:<9} {res['nodes_expanded']:<6}")


if __name__ == "__main__":
    main()
