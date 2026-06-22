import streamlit as st
import pandas as pd

df = pd.read_csv("mesures.csv")

st.title("Mesures balances")
st.line_chart(df["poids"])