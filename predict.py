import pandas as pd
import joblib
import processamento_dados
import sys

ARQUIVO_MODELO = "./model/luva.pkl"
ARQUIVO_COLUNAS = "./model/colunas.pkl"

clf = joblib.load(ARQUIVO_MODELO)
colunas_esperadas = joblib.load(ARQUIVO_COLUNAS)


def prever_movimento(caminho_csv):
    df_novo = pd.read_csv(caminho_csv)
    
    extrator = processamento_dados.Extrator()

    features_dict = extrator.extrair_features(df_novo)
    
    df_features = pd.DataFrame([features_dict])
    
    df_features = df_features.reindex(columns=colunas_esperadas, fill_value=0)
    
    predicao = clf.predict(df_features)[0]
    probabilidade = clf.predict_proba(df_features).max()
    
    return predicao, probabilidade


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Uso: python predict.py <caminho_para_csv>")
        sys.exit(1)

    caminho_csv = sys.argv[1]

    try:
        predicao, probabilidade = prever_movimento(caminho_csv)
        print(f"Predição: {predicao}, Probabilidade: {probabilidade:.2f}")
    except Exception as e:
        print(f"Erro ao realizar a predição: {e}")
        sys.exit(1)
