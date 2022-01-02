
for i in range(0,10):

    # importing all necessary modules 
    from nltk.tokenize import sent_tokenize, word_tokenize 
    import warnings 
    warnings.filterwarnings(action = 'ignore') 
    import gensim 
    from gensim.models import Word2Vec 

    word1 = 'although'
    word2 = 'though'

    # Reads ‘alice.txt’ file 
    sample = open("/Users/chris/Desktop/11-0.txt") 
    s = sample.read() 

    # Replaces escape character with space 
    f = s.replace("\n", " ") 

    data = [] 

    # iterate through each sentence in the file 
    for i in sent_tokenize(f): 
        temp = [] 
        
        # tokenize the sentence into words 
        for j in word_tokenize(i): 
            temp.append(j.lower()) 

        data.append(temp) 

    # Create CBOW model 
    model1 = gensim.models.Word2Vec(data, min_count = 1, 
                                size = 100, window = 5) 

    # Print results 
    print("Cosine similarity between '" + word1 + "' " +
                "and '"+ word2 + "' - CBOW : ", 
        model1.similarity(word1, word2)) 
