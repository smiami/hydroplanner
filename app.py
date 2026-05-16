import streamlit as st
import folium
from folium.plugins import Draw
from streamlit_folium import st_folium
from shapely.geometry import shape, LineString, Polygon, MultiPolygon
import shapely.ops as ops
from shapely.ops import split
import pyproj
import math
import requests
import base64
import urllib3
import geopandas as gpd
import os
import tempfile

# Wyłączenie ostrzeżeń SSL dla bezpiecznego tunelowania do serwerów rządowych
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

st.set_page_config(page_title="HydroPlanner Autonomous Pro", layout="wide")

st.title("⚓ HydroPlanner Enterprise: Interaktywne Cięcie i Multi-Optymalizacja")
st.write("Wersja z natychmiastową synchronizacją kalkulacji profili dla każdego pod-bloku.")

# --- INICJALIZACJA PAMIĘCI SESJI ---
if "drawn_geo" not in st.session_state:
    st.session_state.drawn_geo = None
if "sub_blocks" not in st.session_state:
    st.session_state.sub_blocks = None
if "global_azimuth" not in st.session_state:
    st.session_state.global_azimuth = 0

# --- BAZOWE FUNKCJE MATEMATYCZNE ---
def generate_lines_mesh(poly_obj, azimuth_deg, block_depth, center_lat, center_lon, b_angle, ov):
    crs_wgs84 = pyproj.CRS("EPSG:4326")
    crs_aeqd = pyproj.CRS(f"+proj=aeqd +lat_0={center_lat} +lon_0={center_lon} +datum=WGS84 +units=m")
    trans = pyproj.Transformer.from_crs(crs_wgs84, crs_aeqd, always_xy=True)
    p_meters = ops.transform(trans.transform, poly_obj)
    
    swath = 2 * block_depth * math.tan(math.radians(b_angle / 2))
    sp = swath * (1 - ov)
    rot = 90 - azimuth_deg
    
    rot_poly = ops.transform(lambda x, y: (
        x * math.cos(math.radians(-rot)) - y * math.sin(math.radians(-rot)),
        x * math.sin(math.radians(-rot)) + y * math.cos(math.radians(-rot))
    ), p_meters)
    
    mx, my, xx, xy = rot_poly.bounds
    lines = []
    cy = my + (sp / 2)
    
    while cy <= xy:
        l = LineString([(mx, cy), (xx, cy)])
        inter = l.intersection(rot_poly)
        if not inter.is_empty:
            rest = ops.transform(lambda x, y: (
                x * math.cos(math.radians(rot)) - y * math.sin(math.radians(rot)),
                x * math.sin(math.radians(rot)) + y * math.cos(math.radians(rot))
            ), inter)
            
            if rest.geom_type == 'LineString':
                lines.append(rest)
            elif rest.geom_type in ['MultiLineString', 'GeometryCollection']:
                for sub_geom in rest.geoms:
                    if sub_geom.geom_type == 'LineString' and not sub_geom.is_empty:
                        lines.append(sub_geom)
        cy += sp
    return lines

def find_optimal_azimuth(poly_obj, target_depth, center_lat, center_lon, b_angle, ov, speed, turn):
    best_az = 0
    min_time = float('inf')
    v_ms = speed * 0.51444
    for test_az in range(0, 180, 4):
        test_lines = generate_lines_mesh(poly_obj, test_az, target_depth, center_lat, center_lon, b_angle, ov)
        if len(test_lines) == 0: continue
        dist = sum([l.length for l in test_lines])
        t_hours = ((dist / v_ms) + (len(test_lines) * (turn * 60))) / 3600
        if t_hours < min_time:
            min_time = t_hours
            best_az = test_az
    return best_az

# --- BEZPIECZNE FUNKCJE CALLBACK DLA PRZYCISKÓW ---
def callback_split_bbox():
    if st.session_state.drawn_geo:
        main_poly_obj = shape(st.session_state.drawn_geo)
        c_lat, c_lon = main_poly_obj.centroid.y, main_poly_obj.centroid.x
        
        crs_wgs84 = pyproj.CRS("EPSG:4326")
        crs_aeqd = pyproj.CRS(f"+proj=aeqd +lat_0={c_lat} +lon_0={c_lon} +datum=WGS84 +units=m")
        to_meters = pyproj.Transformer.from_crs(crs_wgs84, crs_aeqd, always_xy=True)
        main_poly_meters = ops.transform(to_meters.transform, main_poly_obj)
        
        minx, miny, maxx, maxy = main_poly_meters.bounds
        if (maxx - minx) > (maxy - miny):
            mid_x = (minx + maxx) / 2
            b1 = Polygon([(minx, miny), (mid_x, miny), (mid_x, maxy), (minx, maxy)])
            b2 = Polygon([(mid_x, miny), (maxx, miny), (maxx, maxy), (mid_x, maxy)])
        else:
            mid_y = (miny + maxy) / 2
            b1 = Polygon([(minx, miny), (maxx, miny), (maxx, mid_y), (minx, mid_y)])
            b2 = Polygon([(minx, mid_y), (maxx, mid_y), (maxx, maxy), (minx, maxy)])
            
        part1, part2 = main_poly_meters.intersection(b1), main_poly_meters.intersection(b2)
        to_gps_trans = pyproj.Transformer.from_crs(crs_aeqd, crs_wgs84, always_xy=True)
        temp_sub_blocks = []
        
        g_dp = st.session_state.get("global_depth_widget", 5.0)
        b_an = st.session_state.get("beam_angle_widget", 120)
        ov = st.session_state.get("overlap_widget", 20) / 100
        sp = st.session_state.get("speed_widget", 3.0)
        tn = st.session_state.get("turn_time_widget", 4.0)
        
        for part in [part1, part2]:
            if not part.is_empty and part.geom_type == 'Polygon':
                gps_shape = ops.transform(to_gps_trans.transform, part)
                opt_az = find_optimal_azimuth(gps_shape, g_dp, c_lat, c_lon, b_an, ov, sp, tn)
                temp_sub_blocks.append({"geo": gps_shape.__geo_interface__, "azimuth": opt_az, "depth": g_dp})
                
        st.session_state.sub_blocks = temp_sub_blocks

def callback_optimize_subblocks():
    if "sub_blocks" in st.session_state and st.session_state.sub_blocks:
        main_poly_obj = shape(st.session_state.drawn_geo)
        c_lat, c_lon = main_poly_obj.centroid.y, main_poly_obj.centroid.x
        
        b_an = st.session_state.get("beam_angle_widget", 120)
        ov = st.session_state.get("overlap_widget", 20) / 100
        sp = st.session_state.get("speed_widget", 3.0)
        tn = st.session_state.get("turn_time_widget", 4.0)
        
        for idx, block in enumerate(st.session_state.sub_blocks):
            block_shape = shape(block["geo"])
            current_b_depth = block["depth"]
            best_az = find_optimal_azimuth(block_shape, current_b_depth, c_lat, c_lon, b_an, ov, sp, tn)
            st.session_state.sub_blocks[idx]["azimuth"] = best_az

def callback_optimize_global():
    if st.session_state.drawn_geo:
        main_poly_obj = shape(st.session_state.drawn_geo)
        c_lat, c_lon = main_poly_obj.centroid.y, main_poly_obj.centroid.x
        
        g_dp = st.session_state.get("global_depth_widget", 5.0)
        b_an = st.session_state.get("beam_angle_widget", 120)
        ov = st.session_state.get("overlap_widget", 20) / 100
        sp = st.session_state.get("speed_widget", 3.0)
        tn = st.session_state.get("turn_time_widget", 4.0)
        
        best_az = find_optimal_azimuth(main_poly_obj, g_dp, c_lat, c_lon, b_an, ov, sp, tn)
        st.session_state["global_azimuth_widget"] = best_az
        st.session_state.global_azimuth = best_az
        st.session_state.sub_blocks = None

# --- PANEL BOCZNY: KONTROLKI SYSTEMOWE ---
with st.sidebar:
    st.header("📂 1. Import Poligonu (SHP)")
    uploaded_files = st.file_uploader("Wgraj pliki Shapefile (.shp, .shx, .dbf, .prj)", type=["shp", "shx", "dbf", "prj"], accept_multiple_files=True)

    st.header("📐 2. Parametry Globalne")
    global_azimuth_input = st.number_input("Globalny Azymut linii [°]", min_value=0, max_value=180, value=int(st.session_state.global_azimuth), step=1, key="global_azimuth_widget")
    global_final_depth = st.number_input("Globalna Głębokość d_min [m]", value=5.0, min_value=0.5, step=0.5, key="global_depth_widget")
    
    st.header("⚙️ 3. Konfiguracja Echosondy")
    beam_angle = st.number_input("Kąt otwarcia MBES [°]", min_value=90, max_value=160, value=120, step=5, key="beam_angle_widget")
    overlap = st.number_input("Nakładanie linii [%]", min_value=10, max_value=50, value=20, step=5, key="overlap_widget") / 100
    
    st.header("🚢 4. Logistyka rejsu")
    speed_knots = st.number_input("Prędkość [węzły]", value=3.0, step=0.5, key="speed_widget")
    turn_time_mins = st.number_input("Czas nawrotu [min]", value=4.0, step=0.5, key="turn_time_widget")
    
    # --- NOWA, WYCZYSZCZONA SEKCJA AKTUALIZACJI IN-PLACE ---
    if st.session_state.sub_blocks:
        st.write("---")
        st.header("🧱 5. Regulacja Pod-Bloków")
        if st.button("🔄 Cofnij podział (Scal z powrotem)"):
            st.session_state.sub_blocks = None
            st.rerun()
            
        for idx in range(len(st.session_state.sub_blocks)):
            with st.expander(f"📦 Pod-Blok {idx+1}", expanded=True):
                # Usunąłem argumenty 'key=' powodujące blokadę zapisu stanów w Streamlit
                sub_az = st.number_input(f"Azymut B{idx+1} [°]", min_value=0, max_value=180, value=int(st.session_state.sub_blocks[idx]["azimuth"]), step=1)
                sub_dp = st.number_input(f"Głębokość B{idx+1} [m]", min_value=0.5, value=float(st.session_state.sub_blocks[idx]["depth"]), step=0.5)
                
                # Bezpośredni, natychmiastowy zapis do data-modelu przed uruchomieniem siatki
                st.session_state.sub_blocks[idx]["azimuth"] = sub_az
                st.session_state.sub_blocks[idx]["depth"] = sub_dp
                
    st.write("---")
    if st.button("🗑️ Resetuj i wyczyść wszystko"):
        st.session_state.drawn_geo = None
        st.session_state.sub_blocks = None
        st.session_state.global_azimuth = 0
        st.rerun()

# --- IMPORT PLIKÓW SHAPEFILE ---
if uploaded_files:
    try:
        with tempfile.TemporaryDirectory() as tmpdir:
            shp_file_path = None
            for uploaded_file in uploaded_files:
                filepath = os.path.join(tmpdir, uploaded_file.name)
                with open(filepath, "wb") as f:
                    f.write(uploaded_file.getbuffer())
                if uploaded_file.name.endswith(".shp"):
                    shp_file_path = filepath
            
            if shp_file_path:
                gdf = gpd.read_file(shp_file_path)
                gdf_wgs84 = gdf.to_crs(epsg=4326)
                geom = gdf_wgs84.geometry.iloc[0]
                if isinstance(geom, (Polygon, MultiPolygon)):
                    st.session_state.drawn_geo = geom.__geo_interface__
                    st.session_state.sub_blocks = None 
                    st.sidebar.success("✅ Zaimportowano plik SHP!")
    except Exception as e:
        st.sidebar.error(f"Błąd SHP: {str(e)}")

# Wyznaczenie współrzędnych startowych mapy
if st.session_state.drawn_geo:
    poly_shape = shape(st.session_state.drawn_geo)
    center_lat, center_lon = poly_shape.centroid.y, poly_shape.centroid.x
    zoom = 14
    wms_min_lon, wms_min_lat, wms_max_lon, wms_max_lat = poly_shape.bounds
    wms_min_lon, wms_max_lon = wms_min_lon - 0.02, wms_max_lon + 0.02
    wms_min_lat, wms_max_lat = wms_min_lat - 0.01, wms_max_lat + 0.01
else:
    center_lat, center_lon = 54.538, 18.568
    zoom = 13
    wms_min_lon, wms_max_lon = 18.490, 18.660
    wms_min_lat, wms_max_lat = 54.510, 54.570

m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom)
folium.TileLayer('openstreetmap', name="Podkład lądu").add_to(m)
folium.TileLayer(tiles="https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png", attr="OpenSeaMap", name="Boje", overlay=True).add_to(m)

# Pobieranie batymetrii WMS Proxy z SIPAM
sipam_image_uri = None
try:
    to_2180 = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:2180", always_xy=True)
    min_x, min_y = to_2180.transform(wms_min_lon, wms_min_lat)
    max_x, max_y = to_2180.transform(wms_max_lon, wms_max_lat)
    wms_params = {
        "SERVICE": "WMS", "VERSION": "1.1.1", "REQUEST": "GetMap", "FORMAT": "image/png",
        "TRANSPARENT": "true", "LAYERS": "mapy_BHMW", "SRS": "EPSG:2180",
        "WIDTH": "1200", "HEIGHT": "1200", "FORMAT_OPTIONS": "dpi:113", "BBOX": f"{min_x},{min_y},{max_x},{max_y}"
    }
    wms_headers = {"User-Agent": "Mozilla/5.0", "Referer": "https://sipam.gov.pl/geoportaldps"}
    res = requests.get("https://sipam.gov.pl/geoserver/ENC/wms", params=wms_params, headers=wms_headers, timeout=10, verify=False)
    if res.status_code == 200 and b"Exception" not in res.content:
        sipam_image_uri = f"data:image/png;base64,{base64.b64encode(res.content).decode('utf-8')}"
except:
    pass

if sipam_image_uri:
    folium.raster_layers.ImageOverlay(image=sipam_image_uri, bounds=[[wms_min_lat, wms_min_lon], [wms_max_lat, wms_max_lon]], opacity=1.0).add_to(m)

# --- PROCESOWANIE I RYSOWANIE GEOMETRII ---
total_distance_global = 0.0
total_lines_global = 0
sub_blocks_data = []

if st.session_state.drawn_geo:
    if st.session_state.sub_blocks:
        blocks_shapes = [shape(b["geo"]) for b in st.session_state.sub_blocks]
        azimuths = [b["azimuth"] for b in st.session_state.sub_blocks]
        depths = [b["depth"] for b in st.session_state.sub_blocks]
        colors = ['#ff0000', '#00ff00', '#ff00ff', '#ffff00']
    else:
        blocks_shapes = [shape(st.session_state.drawn_geo)]
        azimuths = [global_azimuth_input]
        depths = [global_final_depth]
        colors = ['#ff0000']

    crs_wgs84 = pyproj.CRS("EPSG:4326")
    crs_aeqd = pyproj.CRS(f"+proj=aeqd +lat_0={center_lat} +lon_0={center_lon} +datum=WGS84 +units=m")
    to_gps = pyproj.Transformer.from_crs(crs_aeqd, crs_wgs84, always_xy=True)

    for idx, (block_poly, az, b_depth) in enumerate(zip(blocks_shapes, azimuths, depths)):
        # b_depth przesyła się teraz w czasie rzeczywistym bez żadnych opóźnień
        block_lines = generate_lines_mesh(block_poly, az, b_depth, center_lat, center_lon, beam_angle, overlap)
        total_lines_global += len(block_lines)
        color = colors[idx % len(colors)]
        
        for line_m in block_lines:
            total_distance_global += line_m.length
            line_gps = ops.transform(to_gps.transform, line_m)
            
            if line_gps.geom_type == 'LineString':
                folium.PolyLine(locations=[(lat, lon) for lon, lat in line_gps.coords], color=color, weight=3).add_to(m)
            elif line_gps.geom_type in ['MultiLineString', 'GeometryCollection']:
                for sub_l in line_gps.geoms:
                    if sub_l.geom_type == 'LineString':
                        folium.PolyLine(locations=[(lat, lon) for lon, lat in sub_l.coords], color=color, weight=3).add_to(m)
        
        folium.GeoJson(block_poly.__geo_interface__, style_function=lambda x, c=color: {'fillColor': c, 'color': c, 'weight': 2, 'fillOpacity': 0.03}).add_to(m)
        sub_blocks_data.append({"lines": block_lines, "azimuth": az, "depth": b_depth})

if not st.session_state.drawn_geo:
    draw_control = Draw(draw_options={"polyline": False, "rectangle": True, "polygon": True, "circle": False, "marker": False, "circlemarker": False}, edit_options={"remove": True})
    m.add_child(draw_control)
elif st.session_state.drawn_geo and not st.session_state.sub_blocks:
    st.info("✏️ **Tryb cięcia aktywny:** Wybierz ikonę linii (Polyline) po lewej stronie mapy i przetnij nią poligon, aby dokonać podziału.")
    draw_control = Draw(draw_options={"polyline": True, "rectangle": False, "polygon": False, "circle": False, "marker": False, "circlemarker": False}, edit_options={"remove": False})
    m.add_child(draw_control)

folium.LayerControl().add_to(m)

# --- WYŚWIETLANIE INTERFEJSU ---
col_map, col_metrics = st.columns([2, 1])

with col_map:
    map_data = st_folium(m, width=850, height=550, key="master_unified_hydro_map")

with col_metrics:
    st.markdown("### 📊 Autonomiczny Analizator Ekonomiczny")
    
    if st.session_state.drawn_geo:
        main_poly_obj = shape(st.session_state.drawn_geo)
        
        st.markdown("#### 🧩 Krok 1: Segmentacja Geometryczna")
        st.button("Zaproponuj automatyczny podział osiowy (50/50)", on_click=callback_split_bbox)

        st.markdown("#### 🤖 Optymalizacja Kursów Ekonomicznych")
        if st.session_state.sub_blocks:
            st.button("🤖 Autonomicznie zoptymalizuj kurs pod każdy blok osobno", on_click=callback_optimize_subblocks)
        else:
            st.button("Dobierz ekonomiczny azymut globalny", on_click=callback_optimize_global)

        # --- ZBIORCZY RAPORT EKONOMICZNY ---
        if total_lines_global > 0:
            st.write("---")
            st.markdown("#### 📈 Zbiorczy raport ekonomiczny misji:")
            v_ms = speed_knots * 0.51444
            total_time_hours = ((total_distance_global / v_ms) + (total_lines_global * (turn_time_mins * 60))) / 3600
            
            st.metric("Łączna liczba profili (Nawrotów)", f"{total_lines_global} szt.")
            st.metric("Całkowita droga pomiarowa", f"{total_distance_global/1000:.2f} km")
            st.metric("Czas pracy jednostki", f"{total_time_hours:.1f} godz.")
            
            with st.expander("🔍 Szczegółowe parametry poszczególnych pod-bloków", expanded=True):
                for idx, block in enumerate(sub_blocks_data):
                    b_dist = sum([l.length for l in block["lines"]])
                    b_time = ((b_dist / v_ms) + (len(block["lines"]) * (turn_time_mins * 60))) / 3600
                    b_spacing = (2 * block["depth"] * math.tan(math.radians(beam_angle / 2))) * (1 - overlap)
                    st.markdown(f"**Blok {idx+1}**")
                    st.write(f"• Głębokość ($d_{{min}}$): **{block['depth']:.1f} m** $\rightarrow$ Rozstaw: **{b_spacing:.1f} m**")
                    st.write(f"• Azymut roboczy: **{block['azimuth']}°** | Profili: **{len(block['lines'])} szt.** | Czas: **{b_time:.1f} godz.**")
                    st.write("---")

            wkt_all = []
            for idx, block in enumerate(sub_blocks_data):
                wkt_all.append(f"# BLOK {idx+1} - AZYMUT {block['azimuth']} DEG - DEEP {block['depth']} M")
                wkt_all.extend([line.wkt for line in block["lines"]])
            st.download_button("💾 Eksportuj plan linii (.txt)", "\n".join(wkt_all), file_name="indywidualny_plan_misji.txt")
    else:
        st.warning("👈 Wgraj pliki SHP lub narysuj poligon na mapie, aby odblokować algorytmy.")

# --- INTERCYPCJA RYSUNKÓW (Ręczne cięcie) ---
if map_data and map_data.get("all_drawings") and len(map_data["all_drawings"]) > 0:
    for drawing in map_data["all_drawings"]:
        geo = drawing["geometry"]
        
        if geo["type"] in ["Polygon", "Rectangle"] and not st.session_state.drawn_geo:
            st.session_state.drawn_geo = geo
            st.session_state.sub_blocks = None
            st.rerun()
            
        elif geo["type"] == "LineString" and st.session_state.drawn_geo and not st.session_state.sub_blocks:
            cutting_line = shape(geo)
            main_poly = shape(st.session_state.drawn_geo)
            split_result = split(main_poly, cutting_line)
            
            temp_sub_blocks = []
            for part in split_result.geoms:
                if part.geom_type == 'Polygon' and not part.is_empty:
                    opt_az = find_optimal_azimuth(part, global_final_depth, center_lat, center_lon, beam_angle, overlap, speed_knots, turn_time_mins)
                    temp_sub_blocks.append({"geo": part.__geo_interface__, "azimuth": opt_az, "depth": global_final_depth})
            
            if len(temp_sub_blocks) > 1:
                st.session_state.sub_blocks = temp_sub_blocks
                st.rerun()