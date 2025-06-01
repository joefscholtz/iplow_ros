#!/usr/bin/python3

import pandas as pd
from pathlib import Path

current_path = Path(__file__).parent.resolve()

df = pd.read_csv(current_path / "origins.csv",header=0)
cols = ['x', 'y', 'z']

#mm to meters
for col in cols:
    df[col] = df[col]/1000.0

#define base and base_link frames between the wheels
df['y'] = df['y'] - df[df['mesh'] == 'right_wheel']['y'].iloc[0]

#base frame translation
print(df[df['mesh'].isin(['body'])])

df.loc[df[df['mesh'] == 'body'].index,'y'] = 0.0

print(df[df['mesh'].isin(['body'])])

for col in cols:
    df[col] = df[col] - df[df['mesh'] == 'body'][col].iloc[0]

print(df[df['mesh'].isin(['left_wheel', 'right_wheel', 'left_caster_pivot', 'right_caster_pivot', 'mid360'])])

for col in cols:
    df[col] = df[col] - df[df['mesh'] == 'left_caster_pivot'][col].iloc[0]

print(df[df['mesh'].isin(['left_caster_wheel'])])

for col in cols:
    df[col] = df[col] - df[df['mesh'] == 'right_caster_pivot'][col].iloc[0]

print(df[df['mesh'].isin(['right_caster_wheel'])])
