import pandas as pd
import numpy as np
import glob
import processamento_dados
import joblib
from sklearn.metrics import classification_report
from sklearn.model_selection import StratifiedKFold, cross_val_predict, cross_val_score
from sklearn.ensemble import RandomForestClassifier
from scipy.stats import skew, kurtosis

ARQUIVO_MODELO = "./model/luva.pkl"
ARQUIVO_COLUNAS = "./model/colunas.pkl"
PASTA_DADOS_OI = "./training_data/OI/*.csv" 
PASTA_DADOS_PASSA = "./training_data/PASSA/*.csv"
PASTA_DADOS_SOCO = "./training_data/SOCO/*.csv"
COLUNAS_SENSORES = ['qx', 'qy', 'qz', 'qw', 'px', 'py', 'pz']


features = []
labels = []

arquivos_oi = glob.glob(PASTA_DADOS_OI)
arquivos_passa = glob.glob(PASTA_DADOS_PASSA)
arquivos_soco = glob.glob(PASTA_DADOS_SOCO)
print(f"Encontrados {len(arquivos_oi)} arquivos oi para treinamento.")
print(f"Encontrados {len(arquivos_passa)} arquivos passa para treinamento.")
print(f"Encontrados {len(arquivos_soco)} arquivos soco para treinamento.")
pastas = [arquivos_oi, arquivos_soco, arquivos_passa]


for arquivos in pastas:
    for arquivo in arquivos:
        try:
            df = pd.read_csv(arquivo)

            label_atual = df['label'].iloc[0]

            extrator = processamento_dados.Extrator()

            dados_arquivo = extrator.extrair_features(df)
                
            features.append(dados_arquivo)
            labels.append(label_atual)
        except Exception as e:
            print(f"Erro ao ler {arquivo}: {e}")

X = pd.DataFrame(features)
print(X)
y = np.array(labels)
print(y)


clf = RandomForestClassifier(n_estimators=100)

# skf = StratifiedKFold(n_splits=5, shuffle=True) O de producao nao tem k-fold

clf.fit(X, y)

joblib.dump(clf, ARQUIVO_MODELO)

joblib.dump(X.columns.tolist(), ARQUIVO_COLUNAS)

#scores = cross_val_score(clf, X, y, cv=skf, scoring='accuracy')

#print(f"Resultados dos 5 folds: {scores}")
#print(f"Média de Acurácia: {scores.mean():.2f}")
#print(f"Desvio Padrão: {scores.std():.2f}")

