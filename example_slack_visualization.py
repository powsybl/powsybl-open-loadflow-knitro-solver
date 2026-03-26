"""
Example: Extract Slack Information and Visualize with NAD Explorer

This script demonstrates how to:
1. Load a network from XIIDM format
2. Extract slack information (P/Q/V control types) for all voltage levels
3. Enhance nad_explorer with slack visualization
"""

import pypowsybl as pp
import pandas as pd
from pypowsybl_jupyter import nad_explorer
from utils import extract_slack_info_for_nad


def main():
    # =========================================================================
    # Step 1: Load Network
    # =========================================================================
    print("Loading network...")
    network = pp.network.load('data/ieee300.xiidm')
    # Or: network = pp.network.load('data/rte1888.xiidm')
    
    # =========================================================================
    # Step 2: Extract Slack Information
    # =========================================================================
    print("Extracting slack information...")
    slack_info = extract_slack_info_for_nad(network)
    
    print("\nSlack Information Summary:")
    print(slack_info)
    
    print("\nVoltage Levels with Slack:")
    slack_vls = slack_info[slack_info['has_slack'] == True]
    print(f"Total VLs with slack: {len(slack_vls)}")
    print(slack_vls[['has_slack', 'slack_type', 'slack_elements']])
    
    # =========================================================================
    # Step 3: Analyze by Slack Type
    # =========================================================================
    print("\nBreakdown by Slack Control Type:")
    print(f"  P (Active Power): {len(slack_info[slack_info['slack_type'] == 'P'])}")
    print(f"  Q (Reactive Power): {len(slack_info[slack_info['slack_type'] == 'Q'])}")
    print(f"  V (Voltage): {len(slack_info[slack_info['slack_type'] == 'V'])}")
    print(f"  None: {len(slack_info[slack_info['slack_type'] == 'none'])}")
    
    # =========================================================================
    # Step 4: Create Enhanced NAD Explorer with Slack Visualization
    # =========================================================================
    print("\nCreating enhanced NAD explorer...")
    
    # Option A: Simple visualization with all VLs
    explorer = nad_explorer(
        network=network,
        depth=1
    )
    
    # To display in Jupyter:
    # display(explorer)
    
    
    # =========================================================================
    # Step 5: Filter and Analyze Slack VLs
    # =========================================================================
    
    # Get only voltage levels with slack
    slack_vl_ids = slack_info[slack_info['has_slack'] == True].index.tolist()
    print(f"\nVoltage Level IDs with slack: {slack_vl_ids[:10]}...")  # Print first 10
    
    # Create explorer focused on slack voltage levels
    if slack_vl_ids:
        explorer_slack = nad_explorer(
            network=network,
            voltage_level_ids=slack_vl_ids[:5],  # Show first 5 with slack
            depth=1
        )
        # display(explorer_slack)
    
    
    # =========================================================================
    # Step 6: Export slack info to CSV
    # =========================================================================
    print("\nExporting slack information to CSV...")
    slack_export = pd.DataFrame({
        'vl_id': slack_info.index,
        'has_slack': slack_info['has_slack'].values,
        'slack_type': slack_info['slack_type'].values,
        'num_slack_buses': [len(buses) for buses in slack_info['slack_buses'].values],
        'slack_elements': ['; '.join(elem) for elem in slack_info['slack_elements'].values]
    })
    
    slack_export.to_csv('slack_voltage_levels.csv', index=False)
    print("Saved to: slack_voltage_levels.csv")
    
    return network, slack_info, slack_export


if __name__ == '__main__':
    network, slack_info, slack_export = main()
