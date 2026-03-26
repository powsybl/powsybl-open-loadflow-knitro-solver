from math import isnan

import pandas as pd
import pypowsybl.network as pn
import pypowsybl as pp
from pypowsybl_jupyter import nad_explorer
import re
import networkx as nx
import plotly.graph_objects as go
import plotly.express as px
import matplotlib.pyplot as plt
import numpy as np
import builtins
import itertools

def slack_subgraph_2nd(G, data_grouped):
    seen = set()  
    components = []

    for n in G.nodes():
        if not has_slack(G, n, data_grouped):
            continue

        comp = frozenset(dfs_slack_2nd(G, n, data_grouped))

        if comp not in seen:
            seen.add(comp)
            components.append(comp)

    return components

def dfs_slack_2nd(G, source, data_grouped):
    visited = set([source]) # list et pas set ordonnée par précédence pour pouvoir ajouter / retenir ordre de précédence 
    stack = [source] 
    while stack:
        u = stack.pop()

        for v in G.neighbors(u):
            #print(f"Visiting neighbor {v} of {u}")
            if v in visited:
                continue
            
            if has_slack(G, v, data_grouped): 
                visited.add(v)
                stack.append(v)
            #    print(f"Added {v} to visited and stack")
            # if not has_slack(G,v,data_grouped):
            #     z = dfs_with_slack(G, v, data_grouped)
            #     print(f"DFS from {v} found nodes: {z}")
            #     if z - visited:  # Only proceed if there are new nodes in z
            #         visited.update(z)
            #         stack.extend(z)
                    
            #     # visited.add(v)
            #     # stack.append(v)

            if not has_slack(G, v, data_grouped):
                for neighbor in G.neighbors(v):
                    if neighbor in visited:
                        continue
                    if has_slack(G, neighbor, data_grouped):
                        visited.add(neighbor)
                        stack.append(neighbor)
                        ## refaire un 
                      #  visited.add(v)
                        stack.append(v)
                        # print(f"Added {neighbor} to visited and stack from {v}")
                    if not has_slack(G, neighbor, data_grouped):
                        continue
               

    return visited

# Faire une fonction 
def edges_with_impedance(network):
    lines = network.get_lines()
    lines['impedance'] = lines['r'] + 1j * lines['x'] # Z = R +jX 
    lines['impedance_module'] = np.sqrt(lines['r']**2 + lines['x']**2) # |Z| = sqrt(R^2 + X^2)
    transformers_2 = network.get_2_windings_transformers()
    if not transformers_2.empty:
        transformers_2['impedance'] = transformers_2['r'] + 1j * transformers_2['x'] # Z = R +jX
        transformers_2['impedance_module'] = np.sqrt(transformers_2['r']**2 + transformers_2['x']**2) # |Z| = sqrt(R^2 + X^2)
        edges_impedance = pd.concat([lines[['r', 'x', 'impedance', 'impedance_module']],transformers_2[['r', 'x', 'impedance', 'impedance_module']],
        ], ignore_index=False)
    branches = network.get_branches()
    transformers_3 = network.get_3_windings_transformers()
    if not transformers_3.empty:
        transformers_3['impedance'] = transformers_3['r'] + 1j * transformers_3['x'] # Z = R +jX
        transformers_3['impedance_module'] = np.sqrt(transformers_3['r']**2 + transformers_3['x']**2) # |Z| = sqrt(R^2 + X^2)
        edges_impedance = pd.concat([lines[['r', 'x', 'impedance', 'impedance_module']],transformers_3[['r', 'x', 'impedance', 'impedance_module']]
        ], ignore_index=False)
# Combine lines and transformers impedance data
# Merge branches with lines to get r and x values
    edges_list = pd.merge(branches,edges_impedance, left_index=True, right_index=True, how='left')
    return edges_list

# Network construction 
def network_construction(edges_list, data_grouped):
    G = nx.from_pandas_edgelist(edges_list,'bus1_id','bus2_id', edge_attr=True,create_using=nx.Graph())
# ?? nécessaire ? car très lent sur les gros réseaux ?? implémenter que sur les noeds ayant une slack (comme dist)
    nx.set_node_attributes(G, data_grouped['slackValue'],'slack')   
    nx.set_node_attributes(G, data_grouped['slackValue_pu'],'slack_pu')
    nx.set_node_attributes(G, data_grouped['type'],'type')
    nx.set_node_attributes(G, data_grouped['Total_slack_abs'],'Total_slack_abs')
    nx.set_node_attributes(G, data_grouped['gen'],'gen')
    nx.set_node_attributes(G, data_grouped['controlevoltage'],'controlevoltage')
    nx.set_node_attributes(G, data_grouped['transfo'],'transfo')
    nx.set_node_attributes(G, data_grouped['shunt'],'shunt')
    nx.set_node_attributes(G, data_grouped['load'],'load')
  #  nx.set_node_attributes(G, data_grouped['nominal_v'],'V_nom')
    dist = dict(nx.shortest_path_length(G, weight="impedance_module"))
    pos = nx.spring_layout(G, seed=42)

#pertubation = [n for n in G.nodes if n in G_slack.nodes]
    return G, dist, pos

def cluster_subgraph(G, clusters, data_grouped, dist):
    subgraph = {}
    sum_p = {}
    sum_q = {}
    sum_v = {}
    med = {}
    i = 0
    for comp in clusters:
        subgraph[comp] = G.subgraph(comp)
        sum_p[comp] = 0
        sum_q[comp] = 0
        sum_v[comp] = 0
        p_pu = 0
        q_pu =0
        v_pu =0
        p = 0
        q = 0
        v = 0

        data_grouped.loc[list(subgraph[comp].nodes()), 'cluster_size'] = len(comp)
        data_grouped.loc[list(subgraph[comp].nodes()), 'cluster_id'] = i

        for n in subgraph[comp].nodes():
            if n in data_grouped.index:
                node_types = data_grouped.loc[n, 'type']
                slack_values = data_grouped.loc[n, 'slackValue']
                slack_pu = data_grouped.loc[n, 'slackValue_pu']
            
                if isinstance(node_types, list) and isinstance(slack_values, list):
                    for node_type, slack_val, slack_pu in zip(node_types, slack_values, slack_pu):
                        if node_type == 'P':
                            sum_p[comp] += slack_val
                            p_pu += slack_pu
                        elif node_type == 'Q':
                            sum_q[comp] += slack_val
                            q_pu += slack_pu
                        elif node_type == 'V':
                            sum_v[comp] += slack_val
                            v_pu += slack_pu

        if np.abs(sum_p[comp])>np.abs(sum_q[comp]) and np.abs(sum_p[comp])>np.abs(sum_v[comp]):
            data_grouped.loc[list(subgraph[comp].nodes()), 'dominant_slack'] = 'P'
        elif np.abs(sum_q[comp])>np.abs(sum_p[comp]) and np.abs(sum_q[comp])>np.abs(sum_v[comp]):
            data_grouped.loc[list(subgraph[comp].nodes()), 'dominant_slack'] = 'Q'
        elif np.abs(sum_v[comp])>np.abs(sum_p[comp]) and np.abs(sum_v[comp])>np.abs(sum_q[comp]):
            data_grouped.loc[list(subgraph[comp].nodes()), 'dominant_slack'] = 'V'

        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_P'] = sum_p[comp]
        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_Q'] = sum_q[comp]
        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_V'] = sum_v[comp]
        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_P_pu'] = p_pu
        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_Q_pu'] = q_pu
        data_grouped.loc[list(subgraph[comp].nodes()), 'sum_V_pu'] = v_pu

        med[i] = median(subgraph[comp], dist, data=data_grouped)
        data_grouped.loc[list(subgraph[comp].nodes()), 'centroid'] = med[i][0]
        data_grouped.loc[list(subgraph[comp].nodes()), 'centroid_2'] = med[i][1]
        i+=1
            #print(f"Component with {len(comp)} nodes: sum_P={sum_p[comp]:.6f}, sum_Q={sum_q[comp]:.6f}, sum_V={sum_v[comp]:.6f}")
    return data_grouped

def plot_slack_cluster(G, pos, clusters, data_grouped, data, col_centroid):
    # Color cycle for components
   # base_colors = [
    #    "red", "blue", "green", "orange", "purple",
     #   "brown", "cyan", "magenta", "olive", "teal", "pink", "gold", "navy","violet"]
    #colors = itertools.cycle(base_colors)

    fig = go.Figure()

    edge_x_all = []
    edge_y_all = []
    edge_info_all = []

    for u, v, d in G.edges(data=True):
            edge_x_all += [pos[u][0], pos[v][0], None]
            edge_y_all += [pos[u][1], pos[v][1], None]
            txt = f"{d.get('type')}"
            edge_info_all += [txt, None]

    fig.add_trace(go.Scatter(
        x=edge_x_all, y=edge_y_all,
        mode="lines",
        line=dict(width=0.5, color="grey"),
        text=edge_info_all,
        hovertemplate="%{text}<extra></extra>",
        name="edges"
        ))
    
    node_x_all = [pos[n][0] for n in G.nodes() if not has_slack(G, n, data_grouped)]
    node_y_all = [pos[n][1] for n in G.nodes() if not has_slack(G, n, data_grouped)]
    hover_all = [f"Node: {n}" for n in G.nodes() if not has_slack(G, n, data_grouped)]

    fig.add_trace(go.Scatter(
        x=node_x_all, y=node_y_all,
        mode="markers",
        marker=dict(size=6, color="gray"),
        text=hover_all,
        hovertemplate="%{text}<extra></extra>",
        name="All nodes"
        ))

    
    # Color mapping 
    centroid = data_grouped.groupby(col_centroid).apply(
	lambda x: builtins.sum(
		builtins.sum(v) if isinstance(v, list) else v
		for v in x["Total_slack_abs"]
	)
    )
    # node_colors = []
    # prepare normalization bounds
    min_val = centroid.values.min()
    max_val = centroid.values.max()
    range_val = max_val - min_val if max_val != min_val else 1.0

    # for n in centroid.index:
    #     val = centroid[n]
    #     # normalize to [0,1]
    #     norm = (val - min_val) / range_val
    #     norm = max(0.0, min(1.0, norm))
    #     color = px.colors.sample_colorscale("sunsetdark", norm)[0]
      #  centroid.append(color )
   # print(centroid['color'])


    def value2colour(v, colorscale, min_val, range_val): 
        t = (v - min_val) / range_val if range_val > 0 else 0.0 
        t = max(0.0, min(1.0, t)) 
        return px.colors.sample_colorscale(colorscale, t)[0]

    for comp in clusters:
        subgraph = G.subgraph(comp)
        # create a single hover text for the centroid
        #slack_data = [subgraph.nodes[n].get('slack', {}) for n in subgraph.nodes()]
        # sum_p = 0
        # sum_q = 0
        # sum_v = 0
        # p_pu = 0
        # q_pu =0
        # v_pu =0
        for n in subgraph.nodes():
            med = data_grouped.loc[n, 'centroid']
            med_2 = data_grouped.loc[n, 'centroid_2']
            # if n in data_grouped.index:
                # node_types = data_grouped.loc[n, 'type']
                # slack_values = data_grouped.loc[n, 'slackValue']
                # slack_value_pu = data_grouped.loc[n, 'slackValue_pu']
                # if isinstance(node_types, list) and isinstance(slack_values, list) and isinstance(slack_value_pu, list):
                #     for node_type, slack_value, slack_value_pu in zip(node_types, slack_values, slack_value_pu):
                #         if node_type == 'P':
                #             sum_p += slack_value
                #             p_pu += slack_value_pu
                #         if node_type == 'Q':
                #             sum_q += slack_value  
                #             q_pu += slack_value_pu      
                #         if node_type == 'V':
                #             sum_v += slack_value
                #             v_pu += slack_value_pu

        if subgraph.nodes[med].get('controlevoltage') is not None:
                control_str = str(subgraph.nodes[med].get('controlevoltage'))
                type_match = re.search(r'type=(\w+)', control_str).group(1) if re.search(r'type=(\w+)', control_str) else 'N/A'
                controller = re.search(r'controllerElements=([^,\)]+)', control_str).group(1) if re.search(r'controllerElements=([^,\)]+)', control_str) else 'N/A'
                status = re.search(r'mergeStatus=(\w+)', control_str).group(1) if re.search(r'mergeStatus=(\w+)', control_str) else 'N/A'


        # if gen/controlevoltage/transfo/shunt/load = Nan take the second centroid 
        centroid_hover = (
            f"Centroid: {med}<br>"
           # f"slack: {subgraph.nodes[med].get('slack')}<br>"
            #f"type: {subgraph.nodes[med].get('type')}<br>"
            f"Gen: {subgraph.nodes[med].get('gen', 'N/A')}<br>"
           # f"Controlevoltage: {subgraph.nodes[med].get('controlevoltage', 'N/A')}<br>"
            f"Controle voltage type: {type_match}<br>"
            f"Controle voltage location: {controller}<br>"
            f"Controle voltage status: {status}<br>"
            f"Transfo: {subgraph.nodes[med].get('transfo', 'N/A')}<br>"
            f"Shunt: {subgraph.nodes[med].get('shunt', 'N/A')}<br>"
            f"Load: {subgraph.nodes[med].get('load', 'N/A')}<br>"
            f"number of nodes: {len(comp)}<br>"
            f"Nominal V = {subgraph.nodes[med].get('V_nom', 'N/A')} kV<br>"
            f"<br>Total P = {data_grouped.loc[med, 'sum_P']:.4f} MW ({data_grouped.loc[med, 'sum_P_pu']:.2f} p.u)<br>Total Q = {data_grouped.loc[med, 'sum_Q']:.2f} MVar ({data_grouped.loc[med, 'sum_Q_pu']:.2f} p.u)<br>Total V = {data_grouped.loc[med, 'sum_V']:.2f} kV ({data_grouped.loc[med, 'sum_V_pu']:.4f} p.u)"
           )
        

        if np.abs(data_grouped.loc[med, 'sum_P'])>np.abs(data_grouped.loc[med, 'sum_Q']) and np.abs(data_grouped.loc[med, 'sum_P'])>np.abs(data_grouped.loc[med, 'sum_V']):
            colorscale = "Blugrn" 
            colorbar_title = "P p.u"
            pos_x=1.1
        elif np.abs(data_grouped.loc[med, 'sum_Q'])>np.abs(data_grouped.loc[med, 'sum_P']) and np.abs(data_grouped.loc[med, 'sum_Q'])>np.abs(data_grouped.loc[med, 'sum_V']):
            colorscale = "Teal" 
            colorbar_title = "Q p.u"
            pos_x=1.05
        elif np.abs(data_grouped.loc[med, 'sum_V'])>np.abs(data_grouped.loc[med, 'sum_P']) and np.abs(data_grouped.loc[med, 'sum_V'])>np.abs(data_grouped.loc[med, 'sum_Q']):
            colorscale = "Burg" 
            colorbar_title = "V p.u"
            pos_x=1

        val = centroid[med] 
        color = value2colour(val, colorscale, min_val, range_val)
        
        fig.add_trace(go.Scatter(
            x=[pos[med][0]], y=[pos[med][1]],   
            mode="markers",
            marker=dict(
            size=12,
            color=color,
            colorscale=colorscale,
            colorbar=dict(title=colorbar_title, x=pos_x),
            cmin=centroid.values.min(),
            cmax=centroid.values.max(), 
            symbol="star"),
            text=[centroid_hover],
            hovertemplate="%{text}<extra></extra>",
            name="Centroids"
        ))
        if pd.notna(med_2) and med_2 != med:
            centroid_hover_2= (
            f"Second centroid: {med_2}<br>"
            f"Gen: {subgraph.nodes[med_2].get('gen', 'N/A')}<br>"
            # f"Controle voltage type: {type_match}<br>"
            # f"Controle voltage location: {controller}<br>"
            # f"Controle voltage status: {status}<br>"
            f"Transfo: {subgraph.nodes[med_2].get('transfo', 'N/A')}<br>"
            f"Shunt: {subgraph.nodes[med_2].get('shunt', 'N/A')}<br>"
            f"Load: {subgraph.nodes[med_2].get('load', 'N/A')}<br>"
           # f"number of nodes: {len(comp)}<br>"
           # f"<br>Total P = {sum_p:.4f} MW <br>Total Q = {sum_q:.4f} MVar <br>Total V = {sum_v:.4f} kV"
        )
            fig.add_trace(go.Scatter(
            x=[pos[med_2][0]], y=[pos[med_2][1]],   
            mode="markers",
            marker=dict( size=12, color=color, symbol="diamond"),
            text=[centroid_hover_2],
            hovertemplate="%{text}<extra></extra>",
            name="Second centroids"
            ))


        # Edges of this component
        edge_x = []
        edge_y = []

        for u, v in subgraph.edges():
            edge_x += [pos[u][0], pos[v][0], None]
            edge_y += [pos[u][1], pos[v][1], None]

        fig.add_trace(go.Scatter(
            x=edge_x, y=edge_y,
            mode="lines",
            line=dict(width=1, color=color),
            hovertemplate=None
        ))

        # Nodes of this component
        node_x = [pos[n][0] for n in comp]
        node_y = [pos[n][1] for n in comp]
        node_hover = [
            f"{n}<br>slack: {G.nodes[n].get('slack_pu')} p.u <br>type: {G.nodes[n].get('type')}"
            for n in comp
        ]

        fig.add_trace(go.Scatter(
            x=node_x, y=node_y,
            mode="markers",
            marker=dict(size=8, color=color, symbol="circle"),
            text=node_hover,
            hovertemplate="%{text}<extra></extra>",
           # name="component_nodes"
        ))
   
   
    total_p =data_grouped.groupby("centroid")["sum_P"].first().sum()
    total_q = data_grouped.groupby("centroid")["sum_Q"].first().sum()
    total_v = data_grouped.groupby("centroid")["sum_V"].first().sum()
    total_ppu = data_grouped.groupby("centroid")["sum_P_pu"].first().sum()
    total_qpu = data_grouped.groupby("centroid")["sum_Q_pu"].first().sum()
    total_vpu = data_grouped.groupby("centroid")["sum_V_pu"].first().sum()

    fig.add_annotation(
    x=0.98, y=0.98, xref="paper", yref="paper",
    text=f"Slack penalty details <br>total P = {total_p:.2f} MW ({total_ppu:.2f} p.u)<br>total Q = {total_q:.2f} MVar ({total_qpu:.2f} p.u)<br>total V = {total_v:.2f} kV ({total_vpu:.2f} p.u)",
    showarrow=False,
    align="left",
    font=dict(size=12, color="white"),
    bordercolor="black",
    borderwidth=1,
    bgcolor="rgba(0,0,0,0.6)",
    )

    
    fig.update_layout(
        width=900, height=600,
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        margin=dict(l=10, r=10, t=10, b=10),
        showlegend=False
    )
    
    return fig

def plot_slack_components_2nd(G, pos, clusters, data_grouped, col_centroid):
    # Color cycle for components
   # base_colors = [
    #    "red", "blue", "green", "orange", "purple",
     #   "brown", "cyan", "magenta", "olive", "teal", "pink", "gold", "navy","violet"]
    #colors = itertools.cycle(base_colors)

    fig = go.Figure()

    edge_x_all = []
    edge_y_all = []
    edge_info_all = []

    for u, v, d in G.edges(data=True):
            edge_x_all += [pos[u][0], pos[v][0], None]
            edge_y_all += [pos[u][1], pos[v][1], None]
            txt = f"{d.get('type')}"
            edge_info_all += [txt, None]

    fig.add_trace(go.Scatter(
        x=edge_x_all, y=edge_y_all,
        mode="lines",
        line=dict(width=0.5, color="grey"),
        text=edge_info_all,
        hovertemplate="%{text}<extra></extra>",
        name="edges"
        ))
    
    node_x_all = [pos[n][0] for n in G.nodes() if not has_slack(G, n, data_grouped)]
    node_y_all = [pos[n][1] for n in G.nodes() if not has_slack(G, n, data_grouped)]
    hover_all = [f"Node: {n}" for n in G.nodes() if not has_slack(G, n, data_grouped)]

    fig.add_trace(go.Scatter(
        x=node_x_all, y=node_y_all,
        mode="markers",
        marker=dict(size=6, color="gray"),
        text=hover_all,
        hovertemplate="%{text}<extra></extra>",
        name="All nodes"
        ))

    
    # Color mapping 
    centroid = data_grouped.groupby(col_centroid).apply(
	lambda x: builtins.sum(
		builtins.sum(v) if isinstance(v, list) else v
		for v in x["Total_slack_abs"]
	)
    )
    node_colors = []
    # prepare normalization bounds
    min_val = centroid.values.min()
    max_val = centroid.values.max()
    range_val = max_val - min_val if max_val != min_val else 1.0

    for n in centroid.index:
        val = centroid[n]
        # normalize to [0,1]
        norm = (val - min_val) / range_val
        norm = max(0.0, min(1.0, norm))
        color = px.colors.sample_colorscale("sunsetdark", norm)[0]
      #  centroid.append(color )
   # print(centroid['color'])
    def value2colour(v):
        t = (v - min_val) / range_val if range_val > 0 else 0.0
        return px.colors.sample_colorscale("Burg", t)[0]

    node_colors = {centre: value2colour(val)
                  for centre, val in centroid.items()}

    for comp in clusters:
        subgraph = G.subgraph(comp)
        med = median(subgraph, subgraph.nodes(), weight="impedance_module", data=data_grouped)
        color = node_colors[med]
        # create a single hover text for the centroid
        #slack_data = [subgraph.nodes[n].get('slack', {}) for n in subgraph.nodes()]
        sum_p = 0
        sum_q = 0
        sum_v = 0
        for n in subgraph.nodes():
                if n in data_grouped.index:
                    node_types = data_grouped.loc[n, 'type']
                    slack_values = data_grouped.loc[n, 'slackValue']
            
            # Handle list of types and slack values
                    if isinstance(node_types, list) and isinstance(slack_values, list):
                        for node_type, slack_val in zip(node_types, slack_values):
                            if node_type == 'P':
                                sum_p += slack_val
                            if node_type == 'Q':
                                sum_q += slack_val
                            if node_type == 'V':
                                sum_v += slack_val
        if subgraph.nodes[med].get('controlevoltage') is not None:
            control_str = str(subgraph.nodes[med].get('controlevoltage'))
            type_match = re.search(r'type=(\w+)', control_str).group(1) if re.search(r'type=(\w+)', control_str) else 'N/A'
            controller = re.search(r'controllerElements=([^,\)]+)', control_str).group(1) if re.search(r'controllerElements=([^,\)]+)', control_str) else 'N/A'
            status = re.search(r'mergeStatus=(\w+)', control_str).group(1) if re.search(r'mergeStatus=(\w+)', control_str) else 'N/A'

        # if gen/controlevoltage/transfo/shunt/load = Nan take the second centroid 
        centroid_hover = (
            f"Centroid: {med}<br>"
           # f"slack: {subgraph.nodes[med].get('slack')}<br>"
            #f"type: {subgraph.nodes[med].get('type')}<br>"
            f"Gen: {subgraph.nodes[med].get('gen', 'N/A')}<br>"
           # f"Controlevoltage: {subgraph.nodes[med].get('controlevoltage', 'N/A')}<br>"
            f"Controle voltage type: {type_match}<br>"
            f"Controle voltage location: {controller}<br>"
            f"Controle voltage status: {status}<br>"
            f"Transfo: {subgraph.nodes[med].get('transfo', 'N/A')}<br>"
            f"Shunt: {subgraph.nodes[med].get('shunt', 'N/A')}<br>"
            f"Load: {subgraph.nodes[med].get('load', 'N/A')}<br>"
            f"number of nodes: {len(comp)}<br>"
            f"<br>Total P = {sum_p:.4f} MW <br>Total Q = {sum_q:.4f} MVar <br>Total V = {sum_v:.4f} kV"
           )

        fig.add_trace(go.Scatter(
            x=[pos[med][0]], y=[pos[med][1]],   
            mode="markers",
            marker=dict(
            size=12,
            color=color,
            colorscale="Burg",
            colorbar=dict(title="Slack normalized impact"),
            cmin=centroid.values.min(),
            cmax=centroid.values.max(), 
            symbol="star"),
            text=[centroid_hover],
            hovertemplate="%{text}<extra></extra>",
            name="Centroids"
        ))

        # Edges of this component
        edge_x = []
        edge_y = []

        for u, v in subgraph.edges():
            edge_x += [pos[u][0], pos[v][0], None]
            edge_y += [pos[u][1], pos[v][1], None]

        fig.add_trace(go.Scatter(
            x=edge_x, y=edge_y,
            mode="lines",
            line=dict(width=1, color=color),
            hovertemplate=None
        ))

        # Nodes of this component
        node_x = [pos[n][0] for n in comp]
        node_y = [pos[n][1] for n in comp]
        node_hover = [
            f"{n}<br>slack: {G.nodes[n].get('slack_pu')} p.u <br>type: {G.nodes[n].get('type')}"
            for n in comp
        ]

        fig.add_trace(go.Scatter(
            x=node_x, y=node_y,
            mode="markers",
            marker=dict(size=8, color=color, symbol="circle"),
            text=node_hover,
            hovertemplate="%{text}<extra></extra>",
           # name="component_nodes"
        ))
   

    
    fig.update_layout(
        width=900, height=600,
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        margin=dict(l=10, r=10, t=10, b=10),
        showlegend=False
    )
    
    return fig

def median_2(G, nodes, centroid, weight,data):
    centroid_2 = None
    best_cost = float("inf")
    dist = dict(nx.shortest_path_length(G, weight=weight)) #faire que dans les clusters 
    for n in [node for node in nodes if node != centroid]:
        cost =0 
     #   print(n)
      #  if n in data_grouped.index:
      #     print(data_grouped.loc[n,'slackValue'][0])
       #     print(dist[n].values())
        for m in nodes:
            if not np.isnan(dist[m][n]) and m in data.index:
              #  print(f"Distance from {n} to {m}: {dist[n].get(m, 'N/A')}")
              cost += abs(data.loc[m,'Total_slack_abs']) * dist[n][m] #if m != n and not np.isnan(dist[n].get(m, np.nan)))
       # print(cost)
        if cost < best_cost:
            best_cost = cost
            centroid_2 = n
          #  print(n, best_cost)

    return centroid_2

# def median(G, nodes, weight,data):
#     best_node = None
#     best_cost = float("inf")
#     dist = dict(nx.shortest_path_length(G, weight=weight)) # mettre en dehors 
#     for n in nodes:
#         cost =0 
#      #   print(n)
#       #  if n in data_grouped.index:
#       #     print(data_grouped.loc[n,'slackValue'][0])
#        #     print(dist[n].values())
#         for m in nodes:
#             if np.isnan(dist[m][n]) and m not in data.index:
#             # if n not in dist or m not in dist[n]:
#                continue 
#             # if m in data.index:
#               #  print(f"Distance from {n} to {m}: {dist[n].get(m, 'N/A')}")
#             cost += abs(data.loc[m,'Total_slack_abs']) * dist[n][m] #if m != n and not np.isnan(dist[n].get(m, np.nan)))
#        # print(cost)
#         if cost < best_cost:
#             best_cost = cost
#             best_node = n
#           #  print(n, best_cost)

#     return best_node

# def median_strict(G, nodes, weight,data):
#     best_node = None
#     best_cost = float("inf")
#     dist = dict(nx.shortest_path_length(G, weight=weight)) #faire que dans les clusters 
#     for n in nodes:
#         cost =0 
#      #   print(n)
#       #  if n in data_grouped.index:
#       #     print(data_grouped.loc[n,'slackValue'][0])
#        #     print(dist[n].values())
#         for m in nodes:
#             # if no path information is available, skip this pair
#             if n not in dist or m not in dist[n]:
#                 continue
#             if m in data.index:
#                 # accumulate weighted distance only when data exists and path exists
#                 cost += abs(data.loc[m,'Total_slack_abs']) * dist[n][m] #if m != n and not np.isnan(dist[n].get(m, np.nan)))
#        # print(cost)
#         if cost < best_cost:
#             best_cost = cost
#             best_node = n
#           #  print(n, best_cost)

#     return best_node


def median(G, dist,data):
    focal_bus = None
    best_cost = float("inf")
    correction_bus = None
    for n in G.nodes():
        cost =0 
        for m in G.nodes():
            if np.isnan(dist[m][n]) and m not in data.index:
               continue 

            cost += abs(data.loc[m,'Total_slack_abs']) * dist[n][m] #if m != n and not np.isnan(dist[n].get(m, np.nan)))

        if cost < best_cost:
            best_cost = cost
            focal_bus = n

    if data.loc[n,'dominant_slack'] == 'P':
        correction_bus = min((n for n in G.nodes() if n != focal_bus and (pd.notna(type_controller(G, n)[1]).any() or pd.notna(type_controller(G, n)[4]).any())), key=lambda n: dist[focal_bus][n], default=None)
        if (pd.isna(type_controller(G, focal_bus)[1]).all() and pd.isna(type_controller(G, focal_bus)[4]).all()):
           if correction_bus is not None:
               print(f"Focal bus perturbation {focal_bus} has dominant slack P but no suitable controller\n WARNING: NEED TO FIND OTHER CORRECTING BUS, {correction_bus} with distance {dist[focal_bus][correction_bus]} and bus with {type_controller(G, correction_bus)}")
           else :
              print(f"Focal bus perturbation {focal_bus} has dominant slack P but no suitable controller\n WARNING: NO other possible nodes (cluster of 1)")
        else :
            print(f"Focal bus perturbation {focal_bus} is a plossible correcting bus cluster of type P, bus with {type_controller(G, focal_bus)}")

    elif data.loc[n,'dominant_slack'] == 'Q':
        correction_bus = min((n for n in G.nodes() if n != focal_bus and (pd.notna(type_controller(G, n)[1]).any() or pd.notna(type_controller(G, n)[3]).any())), key=lambda n: dist[focal_bus][n], default=None)
        if (pd.isna(type_controller(G, focal_bus)[1]).all() and pd.isna(type_controller(G, focal_bus)[3]).all()):
            print(f"Focal bus perturbation {focal_bus} is a plossible correcting bus, cluster of type Q, bus with {type_controller(G, focal_bus)}")
            if correction_bus is not None:
                print(f"Closest possible correcting bus is {correction_bus} with distance {dist[focal_bus][correction_bus]} and bus with {type_controller(G, correction_bus)}") 
            else :
                print(f"Focal bus perturbation {focal_bus} has dominant slack Q but no suitable controller\n WARNING: NEED TO FIND OTHER CORRECTING BUS, NO other possible nodes (cluster of 1)")
        else :
           # correction_bus = min(dist[focal_bus][n] for n in G.nodes if not pd.isna(type_controller(G, n)[1]).any() or not pd.isna(type_controller(G, n)[3]).any())
            print(f"Focal bus perturbation {focal_bus} has dominant slack Q but no suitable controller \n WARNING: NEED TO FIND OTHER CORRECTING BUS,")# {correction_bus} with distance {dist[focal_bus][correction_bus]} and bus with {type_controller(G, correction_bus)}")
    elif data.loc[n,'dominant_slack'] == 'V':
        correction_bus = min((n for n in G.nodes() if n != focal_bus and (pd.notna(type_controller(G, n)[0]).any() or pd.notna(type_controller(G, n)[1]).any() or pd.notna(type_controller(G, n)[2]).any())), key=lambda n: dist[focal_bus][n], default=None)

        if (pd.isna(type_controller(G, focal_bus)[0]).all() or pd.isna(type_controller(G, focal_bus)[1]).all() or pd.isna(type_controller(G, focal_bus)[2]).all()):
            print(f"Focal bus perturbation {focal_bus} is a plossible correcting bus, cluster of type V, bus with {type_controller(G, focal_bus)}")
            if correction_bus is not None:
                print(f"Closest possible correcting bus is {correction_bus} with distance {dist[focal_bus][correction_bus]} and bus with {type_controller(G, correction_bus)}")
            else :
                print(f"Focal bus perturbation {focal_bus} has dominant slack V but no suitable controller\n WARNING: NEED TO FIND OTHER CORRECTING BUS, NO other possible nodes (cluster of 1)")
        else : 
            print(f"Focal bus perturbation {focal_bus} has dominant slack V but no suitable controller \n WARNING: NEED TO FIND OTHER CORRECTING BUS,")# {correction_bus} with distance {dist[focal_bus][correction_bus]} and bus with {type_controller(G, correction_bus)}")
    else :
        print(f"Focal bus perturbation {focal_bus} has dominant slack {G.nodes[focal_bus].get('dominant_slack')} but no suitable. controller \n WARNING: NEED TO FIND OTHER CORRECTING BUS ")
    return focal_bus, correction_bus

def type_controller(G, n):
    controlevoltage = G.nodes[n].get('controlevoltage')
    gen = G.nodes[n].get('gen')
    transfo = G.nodes[n].get('transfo')
    shunt = G.nodes[n].get('shunt')
    load = G.nodes[n].get('load')
    return controlevoltage, gen, transfo, shunt, load

def slack_subgraph(G, data_grouped):
    seen = set()
    components = []
    for n in G.nodes():
        if not has_slack(G, n, data_grouped):
            continue
        comp = frozenset(dfs_with_slack(G, n, data_grouped))
        if comp not in seen:
            seen.add(comp)
            components.append(comp)


# ============================================================================
# SLACK INFORMATION EXTRACTION FOR VOLTAGE LEVELS
# ============================================================================

def extract_slack_info_for_nad(network):
    """
    Extract slack bus information from a pypowsybl network for visualization.
    
    Returns a DataFrame with columns:
    - vl_id: Voltage Level ID
    - has_slack: Boolean indicating if VL has slack bus
    - slack_type: Type of slack control ('P', 'Q', 'V', or 'none')
    - slack_buses: List of bus IDs with slack in this VL
    - slack_elements: List of generator/battery/compensator IDs
    
    Args:
        network: pypowsybl.network.Network object
        
    Returns:
        DataFrame: Slack information indexed by voltage level ID
    """
    
    # Get all voltage levels
    vls = network.get_voltage_levels(attributes=[])
    
    # Initialize result structure
    slack_data = {
        'vl_id': [],
        'has_slack': [],
        'slack_type': [],
        'slack_buses': [],
        'slack_elements': [],
        'slack_details': []
    }
    
    # Get generators (main slack sources)
    try:
        generators = network.get_generators(attributes=['voltage_level_id', 'target_p', 'target_v', 'target_q', 'bus_id'])
        if generators is not None and not generators.empty:
            gen_by_vl = generators.groupby('voltage_level_id')
        else:
            gen_by_vl = {}
    except:
        gen_by_vl = {}
    
    # Get batteries
    try:
        batteries = network.get_batteries(attributes=['voltage_level_id', 'target_p', 'target_q', 'bus_id'])
        if batteries is not None and not batteries.empty:
            bat_by_vl = batteries.groupby('voltage_level_id')
        else:
            bat_by_vl = {}
    except:
        bat_by_vl = {}
    
    # Get synchronous compensators (can control voltage)
    try:
        sync_comp = network.get_synchronous_compensators(attributes=['voltage_level_id', 'target_v', 'target_q', 'bus_id'])
        if sync_comp is not None and not sync_comp.empty:
            comp_by_vl = sync_comp.groupby('voltage_level_id')
        else:
            comp_by_vl = {}
    except:
        comp_by_vl = {}
    
    # Process each voltage level
    for vl_id in vls.index:
        slack_data['vl_id'].append(vl_id)
        
        has_slack = False
        slack_types = set()
        buses_with_slack = []
        slack_elements_list = []
        slack_details_info = {}
        
        # Check generators in this VL
        if vl_id in gen_by_vl:
            gen_group = gen_by_vl.get_group(vl_id)
            for gen_id, gen_row in gen_group.iterrows():
                has_slack = True
                slack_elements_list.append(f"Gen:{gen_id}")
                bus_id = gen_row.get('bus_id', 'unknown')
                if bus_id and bus_id not in buses_with_slack:
                    buses_with_slack.append(bus_id)
                
                # Determine control type
                # P: if target_p is set and non-zero
                # V: if target_v is set and bus is regulated
                # Q: if target_q is set
                if not pd.isna(gen_row.get('target_p')) and gen_row.get('target_p') != 0:
                    slack_types.add('P')
                if not pd.isna(gen_row.get('target_v')) and gen_row.get('target_v') > 0:
                    slack_types.add('V')
                if not pd.isna(gen_row.get('target_q')):
                    slack_types.add('Q')
                
                slack_details_info[gen_id] = {
                    'type': 'Generator',
                    'target_p': gen_row.get('target_p'),
                    'target_v': gen_row.get('target_v'),
                    'target_q': gen_row.get('target_q')
                }
        
        # Check batteries in this VL
        if vl_id in bat_by_vl:
            bat_group = bat_by_vl.get_group(vl_id)
            for bat_id, bat_row in bat_group.iterrows():
                has_slack = True
                slack_elements_list.append(f"Battery:{bat_id}")
                bus_id = bat_row.get('bus_id', 'unknown')
                if bus_id and bus_id not in buses_with_slack:
                    buses_with_slack.append(bus_id)
                
                if not pd.isna(bat_row.get('target_p')) and bat_row.get('target_p') != 0:
                    slack_types.add('P')
                if not pd.isna(bat_row.get('target_q')):
                    slack_types.add('Q')
                
                slack_details_info[bat_id] = {
                    'type': 'Battery',
                    'target_p': bat_row.get('target_p'),
                    'target_q': bat_row.get('target_q')
                }
        
        # Check synchronous compensators in this VL
        if vl_id in comp_by_vl:
            comp_group = comp_by_vl.get_group(vl_id)
            for comp_id, comp_row in comp_group.iterrows():
                has_slack = True
                slack_elements_list.append(f"SyncComp:{comp_id}")
                bus_id = comp_row.get('bus_id', 'unknown')
                if bus_id and bus_id not in buses_with_slack:
                    buses_with_slack.append(bus_id)
                
                if not pd.isna(comp_row.get('target_v')) and comp_row.get('target_v') > 0:
                    slack_types.add('V')
                if not pd.isna(comp_row.get('target_q')):
                    slack_types.add('Q')
                
                slack_details_info[comp_id] = {
                    'type': 'SynchronousCompensator',
                    'target_v': comp_row.get('target_v'),
                    'target_q': comp_row.get('target_q')
                }
        
        # Determine primary slack type (P has priority as it's the slack bus control)
        slack_type = 'none'
        if slack_types:
            if 'P' in slack_types:
                slack_type = 'P'
            elif 'V' in slack_types:
                slack_type = 'V'
            elif 'Q' in slack_types:
                slack_type = 'Q'
        
        slack_data['has_slack'].append(has_slack)
        slack_data['slack_type'].append(slack_type)
        slack_data['slack_buses'].append(buses_with_slack)
        slack_data['slack_elements'].append(slack_elements_list)
        slack_data['slack_details'].append(slack_details_info)
    
    # Create DataFrame
    df_slack = pd.DataFrame(slack_data)
    df_slack.set_index('vl_id', inplace=True)
    
    return df_slack
    return components

def has_slack(G, v, data_grouped):
    if v in data_grouped.index:
        slack = data_grouped.loc[v, 'Total_slack_abs']
        if slack != 0:
            return True
     #   return not pd.isna(slack)
    else:
        node_slack = G.nodes[v].get('slack', None)
        return node_slack not in (None, 'N/A')

def dfs_with_slack(G, source, data_grouped):
    visited = set([source])
    stack = [source]
    while stack:
        u = stack.pop()

        for v in G.neighbors(u):
            if v in visited:
                continue
            if not has_slack(G, v, data_grouped):
                continue

            visited.add(v)
            stack.append(v)

    return visited

def visualize_graph(G, with_labels=True, k=None, alpha=1.0, node_shape='o'):
    #nx.draw_spring(G, with_labels=with_labels, alpha = alpha)
    pos = nx.spring_layout(G, k=k)
    if with_labels:
        lab = nx.draw_networkx_labels(G, pos, labels=dict([(n, n) for n in G.nodes()]))
    ec = nx.draw_networkx_edges(G, pos, alpha=alpha)
    nc = nx.draw_networkx_nodes(G, pos, nodelist=G.nodes(), node_color='g', node_shape=node_shape)
    plt.axis('off')
def plot_graph(G, pos, highlight):
    hover_line = [f"{u} - {v}<br>type: {d.get('type', 'N/A')}" for u, v, d in G.edges(data=True)]
    hover_text = [f"{n}<br>slack [pu]: {G.nodes[n].get('slack_pu', 'N/A')} <br>type: {G.nodes[n].get('type', 'N/A')}" 
              for n in G.nodes()]
    edge_x = [coord for u, v in G.edges() for coord in (pos[u][0], pos[v][0], None)]
    edge_y = [coord for u, v in G.edges() for coord in (pos[u][1], pos[v][1], None)]

    node_x = [pos[n][0] for n in G.nodes()]
    node_y = [pos[n][1] for n in G.nodes()]

    node_colors = []
    for n in G.nodes():  
        slack = G.nodes[n].get('slack_pu', None)
        if n in highlight:
            node_colors.append('blue')
        elif slack is not None and slack != 'N/A':
            if isinstance(slack, list):
                if any(float(s) != 0 for s in slack if s):
                    node_colors.append('red')
        else:
            node_colors.append('black')
 
    fig = go.Figure([
        go.Scatter(x=edge_x, y=edge_y, mode="lines",
                   line=dict(width=0.5, color='gray'), hoverinfo="text", name="connection", 
                   text=hover_line,
                   hovertemplate='%{text}<extra></extra>'),
        go.Scatter(x=node_x, y=node_y, mode="markers",
                   marker=dict(size=8, color=node_colors), hoverinfo="text", 
                   name="buses", text=hover_text, hovertemplate='%{text}<extra></extra>')
    ])

    fig.update_layout(
        width=800, height=600,
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        margin=dict(l=10, r=10, t=10, b=10),
        showlegend=True
    )
    return fig

def unique_list(series):
    seen = set()
    result = []
    for item in series:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result