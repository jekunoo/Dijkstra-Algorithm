import heapq
from tabulate import tabulate
import matplotlib.pyplot as plt
import networkx as nx
from termcolor import colored

# Fungsi untuk menjalankan algoritma Dijkstra
def dijkstra(graph, start):
    # Menyiapkan jarak awal ke semua node dengan nilai tak terhingga
    distances = {node: float('inf') for node in graph}
    distances[start] = 0  # Jarak dari node awal ke dirinya sendiri adalah 0
    priority_queue = [(0, start)]  # Antrian prioritas untuk memproses node
    previous_nodes = {node: None for node in graph}  # Menyimpan node sebelumnya dalam jalur terpendek
    visited_order = []  # Menyimpan urutan node yang dikunjungi

    while priority_queue:
        # Mengambil node dengan jarak terpendek
        current_distance, current_node = heapq.heappop(priority_queue)

        # Jika jarak saat ini lebih besar dari jarak yang sudah tercatat, lanjutkan
        if current_distance > distances[current_node]:
            continue

        visited_order.append(current_node)  # Catat node yang dikunjungi

        # Mengevaluasi tetangga dari node saat ini
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight  # Menghitung jarak ke tetangga
            if distance < distances[neighbor]:  # Jika jarak baru lebih pendek
                distances[neighbor] = distance  # Update jarak terpendek
                previous_nodes[neighbor] = current_node  # Simpan node sebelumnya
                heapq.heappush(priority_queue, (distance, neighbor))  # Masukkan tetangga ke antrian prioritas

    return distances, previous_nodes, visited_order

# Fungsi untuk menemukan jalur terpendek
def shortest_path(previous_nodes, start, target):
    path = []  # Daftar untuk menyimpan jalur terpendek
    current_node = target

    while current_node != start:  # Selama belum mencapai node awal
        if current_node is None:
            return []  # Tidak ada jalur ditemukan
        path.append(current_node)
        current_node = previous_nodes[current_node]  # Menelusuri node sebelumnya

    path.append(start)  # Tambahkan node awal ke jalur
    path.reverse()  # Balikkan urutan jalur
    return path

# Fungsi untuk memvisualisasikan graf
def visualize_graph(graph, shortest_path=None):
    G = nx.DiGraph()  # Membuat graf terarah
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            G.add_edge(node, neighbor, weight=weight)  # Menambahkan edge dengan bobot

    pos = nx.spring_layout(G)  # Menentukan posisi node dalam visualisasi
    edge_labels = nx.get_edge_attributes(G, 'weight')  # Mendapatkan label bobot pada edge

    plt.figure(figsize=(10, 7))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=12)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')  # Menampilkan label bobot

    if shortest_path:
        path_edges = list(zip(shortest_path, shortest_path[1:]))  # Membuat daftar edge dari jalur terpendek
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='blue', width=2.5)  # Menyoroti jalur terpendek dengan warna biru

    plt.title("Visualisasi Graf dengan Jalur Terpendek Dijkstra")
    plt.show()  # Menampilkan grafik

# Fungsi untuk menghasilkan tabel yang menunjukkan perkembangan algoritma Dijkstra
def generate_table(graph, start, path, previous_nodes):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    visited = set()  # Menggunakan set agar lebih efisien dalam pengecekan node yang sudah dikunjungi
    table_data = []
    step = 1  # Penghitung langkah untuk menandai setiap iterasi

    while priority_queue:
        # Menampilkan langkah saat ini
        print(f"Langkah {step}:")
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node in visited:
            continue  # Lewati node yang sudah dikunjungi

        visited.add(current_node)  # Tandai node ini sudah dikunjungi
        row = {}

        # Menandai node yang sedang diproses dengan warna biru
        for node in graph:
            if distances[node] == float('inf'):
                row[node] = '99'  # Jika node tidak dapat dijangkau, tampilkan '99'
            else:
                previous = previous_nodes[node]
                if previous:
                    value = f"{int(distances[node])}/{previous}"
                    if node == current_node:  # Tandai node yang sedang diproses dengan warna biru
                        row[node] = colored(value, 'blue', attrs=['bold'])
                    else:
                        row[node] = value
                else:
                    value = f"{int(distances[node])}/{start}" if node == start else '99'
                    row[node] = value

        row['Visited'] = current_node
        table_data.append(row)

        # Menampilkan tabel di setiap iterasi
        nodes = list(graph.keys())
        headers = ['Visited'] + nodes
        rows = []
        for row in table_data:
            rows.append([row['Visited']] + [row[node] for node in nodes])

        print(tabulate(rows, headers=headers, tablefmt="grid"))
        print("\n")  # Baris kosong untuk pemisah yang lebih jelas antara iterasi

        # Memperbarui jarak untuk tetangga dan menambahkannya ke antrian prioritas
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

        step += 1  # Menambah langkah setelah setiap iterasi

    return distances, previous_nodes

# Representasi graf contoh
graph = {
    'A': {'B': 6, 'D': 12},
    'B': {'C': 5},
    'C': {'D': 6,'E': 15, 'F': 14 },
    'D': {'E': 8,'F':7},
    'E': {'G': 4},
    'F': {'E': 2, 'I': 10},
    'G': {'I': 6},
    'I': {}
}

start_node = 'A'
target_node = 'I'

# Menjalankan algoritma Dijkstra
distances, previous_nodes, visited_order = dijkstra(graph, start_node)

# Mencari jalur terpendek
path = shortest_path(previous_nodes, start_node, target_node)

# Menampilkan tabel perkembangan
generate_table(graph, start_node, path, previous_nodes)

# Menampilkan hasil jarak terpendek dan jalur terpendek
print(f"Jarak terpendek dari {start_node}: {distances}")
print(f"Jalur terpendek dari {start_node} ke {target_node}: {path}")
print(f"Jalur yang telah dilewati: {visited_order}")

# Memvisualisasikan graf dan jalur terpendek
visualize_graph(graph, shortest_path=path)
