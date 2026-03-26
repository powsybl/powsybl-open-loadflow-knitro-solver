"""
Enhanced NAD Explorer with Slack Information Visualization

Extends the original nad_explorer with slack bus visualization capabilities.
Includes filtering by slack type (P, Q, V) and visual indicators.
"""

import ipywidgets as widgets
import pandas as pd
from pandas import DataFrame
from pypowsybl.network import Network, NadParameters
from pypowsybl_jupyter import nad_explorer as original_nad_explorer
from pypowsybl_jupyter.nadwidget import display_nad, update_nad


def nad_explorer_with_slack(network: Network, 
                            voltage_level_ids: list = None, 
                            depth: int = 1,
                            time_series_data: pd.DataFrame = None, 
                            low_nominal_voltage_bound: float = -1,
                            high_nominal_voltage_bound: float = -1, 
                            parameters: NadParameters = None,
                            fixed_nad_positions: DataFrame = None,
                            slack_info: pd.DataFrame = None):
    """
    
    Args:
        network: the input network
        voltage_level_ids: starting list of VL to display. None displays all VLs
        depth: diagram depth around the voltage level
        time_series_data: DataFrame with time series data
        low_nominal_voltage_bound: low bound to filter VL by nominal voltage
        high_nominal_voltage_bound: high bound to filter VL by nominal voltage
        parameters: NadParameters for layout properties
        fixed_nad_positions: positions dataframe for VL layout
        NEW : slack_info: DataFrame with slack information
                    If None, original nad_explorer behavior is used
    
    Returns:
        NAD explorer with slack info 
    """
    
    vls = network.get_voltage_levels(attributes=[])
    nad_widget = None

    selected_vl = list(vls.index) if voltage_level_ids is None else voltage_level_ids
    if len(selected_vl) == 0:
        raise ValueError("At least one VL must be selected in the voltage_level_ids list")

    if time_series_data is not None:
        time_steps = sorted(time_series_data['timestamp'].unique())
        if len(time_steps) == 0:
            raise ValueError("time_series_data must contain at least one timestamp")
        selected_time_step = time_steps[0]

    selected_depth = depth

    npars = parameters if parameters is not None else NadParameters(
        edge_name_displayed=False,
        id_displayed=False,
        edge_info_along_edge=True,
        power_value_precision=1,
        angle_value_precision=0,
        current_value_precision=1,
        voltage_value_precision=0,
        bus_legend=True,
        substation_description_displayed=True
    )

    # =========================================================================
    # NEW: Process slack information
    # =========================================================================
    slack_dict = {}
    if slack_info is not None:
        slack_dict = {
            row_id: {
                'has_slack': row['has_slack'],
                'slack_type': row['type'][:] if len(row['type']) > 0 else 'none',
                'slack_pu': row['slackValue_pu'][:] if len(row['slackValue_pu']) > 0 else 0,
            }
            for row_id, row in slack_info.iterrows()
        }

    def prepare_branch_states(time_step):
        """
        Prepare branch states data for the selected time step.
        This function extracts the branch data for the given time step and formats it
        for the network-viewer API.
        """
        time_step_data = time_series_data[time_series_data['timestamp'] == time_step]

        branch_states = []
        for _, row in time_step_data.iterrows():
            if 'branch_id' not in row:
                print(f"Warning: 'branch_id' not found in row: {row}")
                continue

            branch_id = row['branch_id']
            branch_state = {
                'branchId': branch_id,
                'value1': float(row.get('value1', 0)),
                'value2': float(row.get('value2', 0)),
                'connected1': bool(row.get('connected1', True)),
                'connected2': bool(row.get('connected2', True)),
            }
            branch_states.append(branch_state)

        if not branch_states:
            print(f"Warning: No branch states found for time step {time_step}")

        return branch_states

    def update_diagram():
        nonlocal nad_widget
        if len(selected_vl) > 0:
            new_diagram_data = network.get_network_area_diagram(
                voltage_level_ids=selected_vl,
                depth=selected_depth,
                high_nominal_voltage_bound=high_nominal_voltage_bound,
                low_nominal_voltage_bound=low_nominal_voltage_bound,
                nad_parameters=npars,
                fixed_positions=fixed_nad_positions
            )
            if nad_widget == None:
                nad_widget = display_nad(new_diagram_data, drag_enabled=True)
            else:
                update_nad(nad_widget, new_diagram_data, drag_enabled=True)

            if time_series_data is not None:
                branch_states = prepare_branch_states(selected_time_step)
                if branch_states:
                    nad_widget.set_branch_states(branch_states)

    nadslider = widgets.IntSlider(
        value=selected_depth, min=0, max=20, step=1, description='depth:',
        disabled=False, continuous_update=False, orientation='horizontal',
        readout=True, readout_format='d'
    )

    def on_nadslider_changed(d):
        nonlocal selected_depth
        selected_depth = d['new']
        update_diagram()

    nadslider.observe(on_nadslider_changed, names='value')

    vl_input = widgets.Text(
        value='', placeholder='Voltage level ID', description='Filter',
        disabled=False, continuous_update=True
    )

    def on_text_changed(d):
        nonlocal selected_vl
        found.options = list(vls[vls.index.str.contains(d['new'], regex=False)].index)
        selected_vl = []

    vl_input.observe(on_text_changed, names='value')

    # =========================================================================
    # NEW: Slack filter dropdown
    # =========================================================================
    slack_filter_options = ['All VLs', 'With Slack', 'Without Slack']
    if slack_info is not None:
        slack_filter_options.extend(['Type: P (Active)', 'Type: Q (Reactive)', 'Type: V (Voltage)'])
    
    slack_filter = widgets.Dropdown(
        options=slack_filter_options,
        value='All VLs',
        description='Slack info:',
        disabled=False,
    )

    def apply_slack_filter(d):
        nonlocal selected_vl
        filter_value = d['new']
        
        if slack_info is None:
            found.options = list(vls.index)
            return
        
        if filter_value == 'All VLs':
            filtered = list(vls.index)
        elif filter_value == 'With Slack':
            filtered = [vl for vl in vls.index if slack_dict.get(vl, {}).get('has_slack', False)]
        elif filter_value == 'Without Slack':
            filtered = [vl for vl in vls.index if not slack_dict.get(vl, {}).get('has_slack', False)]
        elif filter_value == 'Type: P (Active)':
            filtered = [vl for vl in vls.index if slack_dict.get(vl, {}).get('slack_type') == 'P']
        elif filter_value == 'Type: Q (Reactive)':
            filtered = [vl for vl in vls.index if slack_dict.get(vl, {}).get('slack_type') == 'Q']
        elif filter_value == 'Type: V (Voltage)':
            filtered = [vl for vl in vls.index if slack_dict.get(vl, {}).get('slack_type') == 'V']
        else:
            filtered = list(vls.index)
        
        found.options = filtered
        selected_vl = []

    slack_filter.observe(apply_slack_filter, names='value')

    # =========================================================================
    # NEW: Format VL labels with slack indicators
    # =========================================================================
    def format_vl_label_with_slack(vl_id: str) -> str:
        """Format VL label with slack indicator (emoji symbols)."""
        if slack_info is None or vl_id not in slack_dict:
            return vl_id
        
        slack_info_dict = slack_dict[vl_id]
        has_slack = slack_info_dict.get('has_slack', False)
        slack_type = slack_info_dict.get('slack_type', 'none')
        slack_pu = slack_info_dict.get('slack_pu', 0)
        if isinstance(slack_pu, list):
            slack_pu = float(slack_pu[0]) if slack_pu else 0.0
        else:
            slack_pu = float(slack_pu)
        if not has_slack:
            return f"  {vl_id}"
        # # Use emojis to indicate slack type
        # type_map = {'P': 'Active Power', 'Q': '', 'V': '📊'}
        # emoji = type_map.get(slack_type, '●')
        return f" {vl_id} [{slack_type} slack value: {slack_pu:.2f} pu]"

    # Create dropdown with formatted labels
    if slack_info is not None:
        found_options = [(format_vl_label_with_slack(vl), vl) for vl in vls.index]
    else:
        found_options = [(vl, vl) for vl in vls.index]

    found = widgets.SelectMultiple(
        options=found_options,
        value=selected_vl,
        description='Found',
        disabled=False,
        layout=widgets.Layout(height='570px')
    )

    def on_selected(d):
        nonlocal selected_vl
        if d['new'] is not None:
            selected_vl = d['new']
            # Display slack info for selected VLs
            if slack_info is not None:
                slack_display_items = []
                for vl in selected_vl:
                    if vl in slack_dict:
                        info = slack_dict[vl]
                        status = "✓ Slack" if info.get('has_slack') else "✗ No Slack"
                        slack_type = info.get('slack_type', 'none')
                        # slack_value = info.get('slack_pu', 0)
                        # if isinstance(slack_value, list):
                        #     slack_value = float(slack_value[:]) if slack_value else 0.0
                        # else:
                        #     slack_value = float(slack_value)
                        slack_display_items.append(
                            f"<b>{vl}</b>: {status} | Type: {slack_type}"# | Slack Value: {slack_value} pu"
                        )
                if slack_display_items:
                    slack_label.value = '<br>'.join(slack_display_items)
            update_diagram()

    # NEW: Add HTML label to display slack info
    slack_label = widgets.HTML(
        value='<b>Slack Information:</b><br><i>Select voltage levels to see details</i>'
    )

    found.observe(on_selected, names='value')
    update_diagram()

    # =========================================================================
    # NEW: Enhanced layout with slack controls
    # =========================================================================
    left_panel = widgets.VBox([
        widgets.Label('Voltage Levels'),
        slack_filter,
        vl_input,
        widgets.Label('Results:'),
        found,
        widgets.HTML('<hr style="margin: 10px 0;">'),
        slack_label
    ])

    if time_series_data is not None:
        right_panel = widgets.VBox([nadslider, time_slider, nad_widget])
    else:
        right_panel = widgets.VBox([nadslider, nad_widget])

    hbox = widgets.HBox([left_panel, right_panel])
    hbox.layout.align_items = 'flex-start'

    return hbox
