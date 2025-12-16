import pandas as pd
import joblib
import processamento_dados

ARQUIVO_MODELO = "./model/luva.pkl"
ARQUIVO_COLUNAS = "./model/colunas.pkl"

clf = joblib.load(ARQUIVO_MODELO)
colunas_esperadas = joblib.load(ARQUIVO_COLUNAS)


def prever_movimento(caminho_csv):
    """
    Utiliza um csv com padrão conhecido e tenta prever qual movimento pre-determinado foi realizado

    Args:
        caminho_csv (str): O caminho para o arquivo csv

    Returns:
        str: qual foi o movimento predito
        float: confiança na predição
    """
    df_novo = pd.read_csv(caminho_csv)
    
    extrator = processamento_dados.Extrator()

    features_dict = extrator.extrair_features(df_novo)
    
    df_features = pd.DataFrame([features_dict])
    
    df_features = df_features.reindex(columns=colunas_esperadas, fill_value=0)
    
    predicao = clf.predict(df_features)[0]
    probabilidade = clf.predict_proba(df_features).max()
    
    return predicao, probabilidade

